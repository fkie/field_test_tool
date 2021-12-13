#!/usr/bin/env python
"""db2rep.py: Automatic report generator out of the data stored in the FTT database."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import os
import sys
from lxml import etree  # see http://lxml.de/tutorial.html
from jinja2 import Environment, FileSystemLoader  # see http://jinja.pocoo.org
import base64
import time
import datetime
import psycopg2
import psycopg2.sql
import psycopg2.extras
import requests
import parse
import pyproj
import math
from subprocess import call
from PIL import Image, ImageDraw
from StringIO import StringIO

builddir = "../build"
imagedir = "images"
maptiledir = "map_tiles"
logosdir = "../src/logos"
DEFAULT_HEADER_LATEX_TEMPLATE = "rep_header_template.j2"
DEFAULT_SHIFT_HEADER_TEMPLATE = "rep_shift_header_template.j2"
DEFAULT_SHIFT_STATISTICS_HEADER_TEMPLATE = "rep_header_stat_template.j2"
DEFAULT_SHIFT_STATISTICS_DATA_TEMPLATE = "rep_data_stat_template.j2"
DEFAULT_SEGMENT_HEADER_TEMPLATE = "rep_segment_header_template.j2"
TILE_SERVER = ""
ZOOM_LEVEL = 19

###############################################################

class PgAdapter:
    def __init__(self, host, dbname, user, password):
        self.host = host
        self.dbname = dbname
        self.user = user
        self.password = password

    def connect(self):
        conn_string = "host='%s' dbname='%s' user='%s' password='%s'" % (
            self.host, self.dbname, self.user, self.password)
        print ("Connecting to db: %s" % (self.dbname,))
        self.conn = psycopg2.connect(conn_string)
        self.cursor = self.conn.cursor()
        self.dictcursor = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)

class FttReportGenerator:
    def __init__(self, db_adapter, env):
        self.db_adapter = db_adapter
        self.env = env

    def conv_time(self, seconds):
        m, s = divmod(seconds, 60)
        h, m = divmod(m, 60)
        return "%d:%02d:%02d" % (h, m, s)

    @staticmethod
    def webmercator2tiles(easting, northing, zoom):
        # Transform web mercator's projection coordinates (in meters) to tile server's coordinates (with decimal points here).
        n = 2.0 ** zoom
        equator_perimeter = 40075016.68557849
        xtile = (0.5 + easting / equator_perimeter) * n
        ytile = (0.5 - northing / equator_perimeter) * n
        return (xtile, ytile)

    @staticmethod
    def tiles2webmercator(xtile, ytile, zoom):
        # Transform tile server's coordinates to web mercator's projection (in meters).
        n = 2.0 ** zoom
        equator_perimeter = 40075016.68557849
        easting = (xtile / n - 0.5) * equator_perimeter
        northing = (0.5 - ytile / n) * equator_perimeter
        return (easting, northing)

    @staticmethod
    def tile_px_scale(pixels_per_tile, zoom):
        # Get server's tile scale (in meters), according to web mercator's transform.
        n = 2.0 ** zoom
        equator_perimeter = 40075016.68557849
        return pixels_per_tile * n / equator_perimeter

    @staticmethod
    def download_map_tiles(xmin, ymin, xmax, ymax, zoom):
        # Return if there's no tile server assigned.
        if not TILE_SERVER:
            print("No server specified to download map tiles.")
            return     
        # Request tiles from server.
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                # Construct filename string.
                filename = "%s/%s/%s_%s_%s.png" % (builddir, maptiledir, zoom, xtile, ytile)
                if not os.path.isfile(filename):
                    try:
                        # Construct URL string.
                        url_format = r"http://{0}/{1}/{2}/{3}.png"
                        imgurl=url_format.format(TILE_SERVER, zoom, xtile, ytile)
                        print("Opening: " + imgurl)
                        # Read response.
                        response = requests.get(imgurl, headers={"user-agent":"Custom user agent"})
                        imgstr = response.content
                        # Save to file.
                        f_image = open(filename, 'w')
                        f_image.write(imgstr)
                        f_image.close()
                    except:
                        e = sys.exc_info()[0] 
                        print("Couldn't download tile %s/%s/%s" % (zoom, xtile, ytile))
                        print("Error: %s" % e)
        return

    @staticmethod
    def merge_map_tiles(xmin, ymin, xmax, ymax, zoom):
        # Create image canvas.
        map_images = Image.new('RGB',((xmax-xmin+1)*256, (ymax-ymin+1)*256))
        # Find downloaded map tiles and paste them in the canvas.
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                filename = "%s/%s/%s_%s_%s.png" % (builddir, maptiledir, zoom, xtile, ytile)
                try:
                    tile = Image.open(filename)
                    map_images.paste(tile, box=((xtile-xmin)*256 ,  (ytile-ymin)*256))
                except:
                    print("Tile not found: %s/%s/%s" % (zoom, xtile, ytile))
        return map_images

    def draw_segment_points(self, seg_id, map_images, xmin, ymin, zoom):
        # Define object for drawing over the map image.
        draw = ImageDraw.Draw(map_images)
        # Get this and next segment's starting point (in web mercator projection used for the tiles) 
        # and this segment type color from database.
        sel_stmt = "SELECT ST_X(ST_TRANSFORM(this.start_position, 3857)), \
                        ST_Y(ST_TRANSFORM(this.start_position, 3857)), \
                        ST_X(ST_TRANSFORM(next.start_position, 3857)), \
                        ST_Y(ST_TRANSFORM(next.start_position, 3857)), \
                        segment_type.color \
                    FROM segment AS this \
                    LEFT OUTER JOIN segment AS next ON next.id > this.id AND next.leg_id = this.leg_id AND next.parent_id IS NULL \
                    LEFT OUTER JOIN segment_type ON this.segment_type_id = segment_type.id \
                    WHERE this.id = %s \
                    ORDER BY next.id \
                    LIMIT 1" % seg_id
        self.db_adapter.dictcursor.execute(sel_stmt)
        data = self.db_adapter.dictcursor.fetchone()
        start_easting = data[0]
        start_northing = data[1]
        end_easting = data[2]
        end_northing = data[3]
        color = tuple([int(i) for i in data[4].split(',')])
        # Get segment points from database (in web mercator projection).
        sel_stmt = "SELECT ST_X(ST_TRANSFORM(pose.position, 3857)), ST_Y(ST_TRANSFORM(pose.position, 3857)) \
                    FROM pose \
                    INNER JOIN segment ON pose.segment_id = segment.id \
                    WHERE segment_id = %s \
                    ORDER BY pose.id" % seg_id
        self.db_adapter.dictcursor.execute(sel_stmt)
        points = self.db_adapter.dictcursor.fetchall()
        # Return if the segment has no data points.
        if not (start_easting and start_northing or points):
            return
        # Build tuple array out of position data.
        seg_xy = []
        if start_easting and start_northing:
            seg_xy.append((start_easting, start_northing))
        for point in points:
            if point and point[0] and point[1]:
                seg_xy.append((point[0], point[1]))
        if end_easting and end_northing:
            seg_xy.append((end_easting, end_northing))
        # Get the web mercator projection coordinates of the top left tile corner
        # (to use as origin for the position data in the image reference system).
        o_easting, o_northing = FttReportGenerator.tiles2webmercator(xmin, ymin, zoom)
        # Transform coordinates to image reference system.
        scale = FttReportGenerator.tile_px_scale(256, zoom)
        seg_xy = [(scale * (point[0] - o_easting), scale * (o_northing - point[1])) for point in seg_xy]
        # Draw polyline on image.
        if len(seg_xy) > 1:
            draw.line(seg_xy, color, 1)
        # Draw a 3px wide point on each position.
        for p in seg_xy:
            draw.ellipse([p[0]-1, p[1]-1, p[0]+1, p[1]+1], color, color)

    def get_map(self, shift_id, seg_id, bb):
        # Get the bounding box coordinates.
        lon = bb[0::2]
        lat = bb[1::2]
        # Find the bounding box in the Mercator projection EPSG:3857 (used by the tile server).
        wgs84 = pyproj.Proj("+init=EPSG:4326")  # Database's LatLon coordinates.
        mercator = pyproj.Proj("+init=EPSG:3857")  # Tile server projection.
        easting, northing = pyproj.transform(wgs84, mercator, lon, lat)
        # Expand the bounding box by 10% of the longest dimension (at least 10m)
        border = max((easting[1] - easting[0]) / 10, (northing[1] - northing[0]) / 10, 10)
        easting[0] = easting[0] - border
        easting[1] = easting[1] + border
        northing[0] = northing[0] - border
        northing[1] = northing[1] + border
        # Select the proper zoom level (the maximum is always used here).
        zoom = ZOOM_LEVEL
        # Find the coordinates in the tile's reference system (with decimal points).
        xmin_f, ymax_f = FttReportGenerator.webmercator2tiles(easting[0], northing[0], zoom)
        xmax_f, ymin_f = FttReportGenerator.webmercator2tiles(easting[1], northing[1], zoom)
        # Get the needed tile numbers (integer values).
        xmin = int(xmin_f)
        ymin = int(ymin_f)
        xmax = int(xmax_f)
        ymax = int(ymax_f)
        # Get the normalized distance between the original bounding box and the tiles' bounding box.
        xmin_excess = xmin_f - xmin # Distance between left edges.
        ymin_excess = ymin_f - ymin # Distance between top edges.
        xmax_excess = 1 - (xmax_f - xmax) # Since the decimal part is the distance to the left, it's complement is the distance to the right.
        ymax_excess = 1 - (ymax_f - ymax) # Since the decimal part is the distance to the top, it's complement is the distance to the bottom.
        # Get tiles from server and save them to files.
        FttReportGenerator.download_map_tiles(xmin, ymin, xmax, ymax, zoom)
        # Build the map image from the downloaded tiles.
        map_images = FttReportGenerator.merge_map_tiles(xmin, ymin, xmax, ymax, zoom)
        # Add position data to image:
        if seg_id:
            # Draw segment points on image.
            self.draw_segment_points(seg_id, map_images, xmin, ymin, zoom)
        else:
            # Get all master segment ids for the shift
            sel_stmt = "SELECT segment.id \
                        FROM segment \
                        INNER JOIN leg ON segment.leg_id = leg.id \
                        INNER JOIN shift on leg.shift_id = shift.id \
                        WHERE shift.id = %s AND segment.parent_id IS NULL \
                        ORDER BY segment.id" % shift_id
            self.db_adapter.dictcursor.execute(sel_stmt)
            segment_ids = [x[0] for x in self.db_adapter.dictcursor.fetchall()]
            # Draw points on image for all the segments.
            for segment_id in segment_ids:
                self.draw_segment_points(segment_id, map_images, xmin, ymin, zoom)
        # Parametrize cropping to proportionally cut borders of the merged map image up to a minimum size of 256 x 256 px.
        h_crop_factor = max(0, min((map_images.width / 256 - 1) / (xmin_excess + xmax_excess), 1))
        v_crop_factor = max(0, min((map_images.height / 256 - 1) / (ymin_excess + ymax_excess), 1))
        # Crop excess size.
        map_images = map_images.crop((
            int(h_crop_factor * xmin_excess * 256), 
            int(v_crop_factor * ymin_excess * 256), 
            int(map_images.width - h_crop_factor * xmax_excess * 256), 
            int(map_images.height - v_crop_factor * ymax_excess * 256)
        ))
        # Define file name.
        if seg_id:
            file_dir = "%s/%s/segment_%s.jpeg" % (builddir, imagedir, seg_id)
        else:
            file_dir = "%s/%s/shift_%s.jpeg" % (builddir, imagedir, shift_id)
        # Save image to file.
        f_image = open(file_dir, 'w')
        map_images.save(f_image, "JPEG")
        f_image.close()
        return

    def draw_segment_local_points(self, seg_id, map_image, origin_x, origin_y, resolution):
        # Define object for drawing over the map image.
        draw = ImageDraw.Draw(map_image)
        # Get this and next segment's local starting point
        # and this segment type color from database.
        sel_stmt = "SELECT ST_X(this.local_start_position), \
                        ST_Y(this.local_start_position), \
                        ST_X(next.local_start_position), \
                        ST_Y(next.local_start_position), \
                        segment_type.color \
                    FROM segment AS this \
                    LEFT OUTER JOIN segment AS next ON next.id > this.id AND next.leg_id = this.leg_id AND next.parent_id IS NULL \
                    LEFT OUTER JOIN segment_type ON this.segment_type_id = segment_type.id \
                    WHERE this.id = %s \
                    ORDER BY next.id \
                    LIMIT 1" % seg_id
        self.db_adapter.dictcursor.execute(sel_stmt)
        data = self.db_adapter.dictcursor.fetchone()
        start_easting = data[0]
        start_northing = data[1]
        end_easting = data[2]
        end_northing = data[3]
        color = tuple([int(i) for i in data[4].split(',')])
        # Get segment local points from database.
        sel_stmt = "SELECT ST_X(local_pose.position), ST_Y(local_pose.position) \
                    FROM local_pose \
                    INNER JOIN segment ON local_pose.segment_id = segment.id \
                    WHERE segment_id = %s \
                    ORDER BY local_pose.id" % seg_id
        self.db_adapter.dictcursor.execute(sel_stmt)
        points = self.db_adapter.dictcursor.fetchall()
        # Return if the segment has no data points.
        if not (start_easting and start_northing or points):
            return
        # Build tuple array out of position data.
        seg_xy = []
        if start_easting and start_northing:
            seg_xy.append((start_easting, start_northing))
        for point in points:
            if point and point[0] and point[1]:
                seg_xy.append((point[0], point[1]))
        if end_easting and end_northing:
            seg_xy.append((end_easting, end_northing))
        # Transform coordinates to image reference system.
        seg_xy = [((point[0] - origin_x) / resolution, map_image.height - (point[1] - origin_y) / resolution) for point in seg_xy]
        # Draw polyline on image.
        if len(seg_xy) > 1:
            draw.line(seg_xy, color, 1)
        # Draw a 3px wide point on each position.
        for p in seg_xy:
            draw.ellipse([p[0]-1, p[1]-1, p[0]+1, p[1]+1], color, color)

    def get_local_map(self, shift_id, seg_id, bb):
        # Get the bounding box coordinates.
        easting = bb[0::2]
        northing = bb[1::2]
        # Expand the bounding box by 10% of the longest dimension (at least 10m)
        border = max((easting[1] - easting[0]) / 10, (northing[1] - northing[0]) / 10, 10)
        easting[0] = easting[0] - border
        easting[1] = easting[1] + border
        northing[0] = northing[0] - border
        northing[1] = northing[1] + border
        # Get the local map image from the database.
        sel_stmt = "SELECT resolution, ST_X(origin) as origin_x, ST_Y(origin) as origin_y, image_data as image \
            FROM map_image \
            WHERE shift_id = %s"
        self.db_adapter.dictcursor.execute(sel_stmt, (shift_id,))
        map_data = self.db_adapter.dictcursor.fetchone()
        resolution = map_data["resolution"]
        origin_x = map_data["origin_x"]
        origin_y = map_data["origin_y"]
        map_image = Image.open(StringIO(base64.standard_b64decode(map_data["image"]))).convert("RGB")
        # Add local position data to image:
        if seg_id:
            # Draw segment points on image.
            self.draw_segment_local_points(seg_id, map_image, origin_x, origin_y, resolution)
        else:
            # Get all master segment ids for the shift
            sel_stmt = "SELECT segment.id \
                        FROM segment \
                        INNER JOIN leg ON segment.leg_id = leg.id \
                        INNER JOIN shift on leg.shift_id = shift.id \
                        WHERE shift.id = %s AND segment.parent_id IS NULL \
                        ORDER BY segment.id" % shift_id
            self.db_adapter.dictcursor.execute(sel_stmt)
            segment_ids = [x[0] for x in self.db_adapter.dictcursor.fetchall()]
            # Draw points on image for all the segments.
            for segment_id in segment_ids:
                self.draw_segment_local_points(segment_id, map_image, origin_x, origin_y, resolution)
        # Get the distance between the original bounding box and the map image borders (in pixels).
        xmin_excess = (easting[0] - origin_x) / resolution # Distance between left edges.
        ymin_excess = (northing[0] - origin_y) / resolution # Distance between top edges.
        xmax_excess = map_image.width - (easting[1] - origin_x) / resolution # Distance between right edges.
        ymax_excess = map_image.height - (northing[1] - origin_y) / resolution # Distance between bottom edges.
        # Parametrize cropping to proportionally cut borders of the map image up to a minimum size of 256 x 256 px.
        h_crop_factor = max(0, min((map_image.width - 256) / (xmin_excess + xmax_excess), 1))
        v_crop_factor = max(0, min((map_image.height - 256) / (ymin_excess + ymax_excess), 1))
        # Crop excess size.
        map_image = map_image.crop((
            int(h_crop_factor * xmin_excess), 
            int(v_crop_factor * ymax_excess), 
            int(map_image.width - h_crop_factor * xmax_excess), 
            int(map_image.height - v_crop_factor * ymin_excess)
        ))
        # Define file name.
        if seg_id:
            file_dir = "%s/%s/segment_%s.jpeg" % (builddir, imagedir, seg_id)
        else:
            file_dir = "%s/%s/shift_%s.jpeg" % (builddir, imagedir, shift_id)
        # Save image to file.
        f_image = open(file_dir, 'w')
        map_image.save(f_image, "JPEG")
        f_image.close()
        return

    def get_segment_distance(self, segment_id, leg_id, local):
        # Get distance between segment start position and first location point.
        if local:
            select_stmt = "SELECT st_distance (segment.local_start_position, local_pose.position) \
                        FROM segment \
                        INNER JOIN local_pose ON local_pose.segment_id = segment.id \
                        WHERE segment.id = %s \
                        ORDER BY local_pose.secs \
                        LIMIT 1" % segment_id
        else:  
            select_stmt = "SELECT st_distance (st_setsrid(segment.start_position,4326), st_setsrid(pose.position,4326), true) \
                        FROM segment \
                        INNER JOIN pose ON pose.segment_id = segment.id \
                        WHERE segment.id = %s \
                        ORDER BY pose.secs \
                        LIMIT 1" % segment_id
        self.db_adapter.dictcursor.execute(select_stmt)
        data = self.db_adapter.dictcursor.fetchone()
        if data and data[0]:
            start_distance = data[0]
        else:
            start_distance = 0
        # Get distance between location points.
        if local:
            select_stmt = "SELECT \
                    SUM (st_distance (this.position, next.position)) \
                    FROM local_pose as this, local_pose as next \
                    WHERE this.id+1 = next.id  \
                        AND this.segment_id = next.segment_id \
                        AND this.segment_id = %s" % segment_id
        else:
            select_stmt = "SELECT \
                    SUM (st_distance (st_setsrid(this.position,4326), st_setsrid(next.position,4326), true)) \
                    FROM pose as this, pose as next \
                    WHERE this.id+1 = next.id  \
                        AND this.segment_id = next.segment_id \
                        AND this.segment_id = %s" % segment_id
        self.db_adapter.dictcursor.execute(select_stmt)
        data = self.db_adapter.dictcursor.fetchone()
        if data and data[0]:
            distance = data[0]
        else:
            distance = 0
        # Get distance between last location point and next segment's start position.
        if local:
            select_stmt = "SELECT st_distance (segment.local_start_position, local_pose.position) \
                    FROM local_pose \
                    LEFT OUTER JOIN segment ON segment.id = ( \
                        SELECT MIN(id) \
                        FROM segment \
                        WHERE id > %s AND parent_id IS NULL AND leg_id = %s) \
                    WHERE local_pose.id = ( \
                        SELECT MAX(id) \
                        FROM local_pose \
                        WHERE segment_id = %s)" % (segment_id, leg_id, segment_id)
        else:
            select_stmt = "SELECT st_distance (st_setsrid(segment.start_position,4326), st_setsrid(pose.position,4326), true) \
                    FROM pose \
                    LEFT OUTER JOIN segment ON segment.id = ( \
                        SELECT MIN(id) \
                        FROM segment \
                        WHERE id > %s AND parent_id IS NULL AND leg_id = %s) \
                    WHERE pose.id = ( \
                        SELECT MAX(id) \
                        FROM pose \
                        WHERE segment_id = %s)" % (segment_id, leg_id, segment_id)
        self.db_adapter.dictcursor.execute(select_stmt)
        data = self.db_adapter.dictcursor.fetchone()
        if data and data[0]:
            end_distance = data[0]
        else:
            end_distance = 0
        # Add distances.
        distance = start_distance + distance + end_distance
        return distance

    def get_segment_duration(self, segment_id):
        select_stmt = "SELECT endtime_secs - starttime_secs \
            FROM segment \
            WHERE segment.id = %s" % segment_id
        self.db_adapter.dictcursor.execute(select_stmt)
        data = self.db_adapter.dictcursor.fetchone()
        if data and data[0]:
            duration = data[0]
        else:
            duration = 0
        return duration

    def get_shifts_by_test_event_id(self, test_event_id, local):
        select_stmt = psycopg2.sql.SQL("SELECT shift.id as id, \
            to_timestamp(min(segment.starttime_secs)) as shift_start_datetime_raw,\
            to_char(to_timestamp(min(segment.starttime_secs)), 'YYYY-MM-DD HH24:MI:SS') as shift_start_datetime, \
            to_char(to_timestamp(max(segment.endtime_secs)), 'YYYY-MM-DD HH24:MI:SS') as shift_end_datetime, \
            st_extent (pose.position) as shift_bb, \
            user_test_administrator.name as test_administrator_name, \
            user_test_director.name as test_director_name, \
            user_safety_officer.name as safety_officer_name, \
            RTRIM(performer.institution) as performer_institution, \
            RTRIM(shift.test_intent) as test_intent, \
            shift.note as shift_note \
            FROM {} as pose \
            INNER JOIN segment ON pose.segment_id = segment.id \
            INNER JOIN leg ON segment.leg_id = leg.id \
            INNER JOIN shift on leg.shift_id = shift.id \
            LEFT OUTER JOIN personnel as user_test_administrator on \
            shift.test_administrator_id = user_test_administrator.id \
            LEFT OUTER JOIN personnel as user_test_director on shift.test_director_id = user_test_director.id \
            LEFT OUTER JOIN personnel as user_safety_officer on shift.safety_officer_id = user_safety_officer.id \
            LEFT OUTER JOIN performer on shift.performer_id = performer.id \
            WHERE test_event_id = %s \
            GROUP BY shift.id, test_administrator_name, test_director_name, safety_officer_name, performer_institution \
            ORDER BY shift.id").format(psycopg2.sql.Identifier("local_pose" if local else "pose"))
        self.db_adapter.dictcursor.execute(select_stmt, (test_event_id,))
        shifts = self.db_adapter.dictcursor.fetchall()
        return shifts

    def get_shift_timeline(self, shift_id):
        select_stmt = "SELECT segment.id as segment_id, segment.parent_id as segment_parent_id, \
                segment_type.key as segment_type_key, segment_type.short_description as \
                segment_type_short_description, segment_type.color as segment_color, \
                to_timestamp(segment.starttime_secs) as starttime,  \
                to_timestamp(segment.endtime_secs) as endtime,  \
                to_timestamp(segment.endtime_secs) - to_timestamp(segment.starttime_secs) as duration, \
                segment.parent_id,  \
                ito_reason.key as ito_key, \
                ito_reason.color as ito_color, \
                ito_reason.short_description as ito_description, \
                weather.icon_filename as weather_icon_filename \
            FROM segment \
            INNER JOIN leg ON segment.leg_id = leg.id  \
            LEFT OUTER JOIN segment_type on segment.segment_type_id = segment_type.id \
            LEFT OUTER JOIN ito_reason on segment.ito_reason_id = ito_reason.id \
            LEFT OUTER JOIN weather on leg.weather_id = weather.id \
            WHERE leg.shift_id = %s \
            ORDER BY segment.starttime_secs"
        self.db_adapter.dictcursor.execute(select_stmt, (shift_id,))
        shift_timeline = self.db_adapter.dictcursor.fetchall()
        return shift_timeline

    def get_shift_statistics(self, shift_id, local):
        # Get segment types.
        select_stmt = "SELECT id, short_description \
            FROM segment_type"
        self.db_adapter.dictcursor.execute(select_stmt)
        data = self.db_adapter.dictcursor.fetchall()
        segment_types = [(row[0], row[1]) for row in data]
        shift_statistics = []
        # Get statistics for each segment type.
        for segment_type_id, segment_type_short_description in segment_types:
            stats = {'segment_type_short_description': segment_type_short_description}
            # Get master segment ids.
            select_stmt = "SELECT segment.id, leg_id \
                FROM segment \
                INNER JOIN leg ON segment.leg_id = leg.id \
                WHERE leg.shift_id = %s \
                    AND segment.segment_type_id = %s \
                    AND segment.parent_id IS NULL" % (shift_id, segment_type_id)
            self.db_adapter.dictcursor.execute(select_stmt)
            data = self.db_adapter.dictcursor.fetchall()
            # Get id count.
            stats['cnt'] = len(data)
            # Get total segment distance.
            stats['distance'] = sum([self.get_segment_distance(row[0], row[1], local) for row in data])
            # Get total segment duration.
            stats['duration'] = sum([self.get_segment_duration(row[0]) for row in data])
            # Add segment type statistics to result.
            shift_statistics.append(stats)

        return shift_statistics

    @staticmethod
    def get_bb_from_text(bb_str):
        bb = parse.parse("BOX({} {},{} {})", bb_str)
        bb = map(float, bb)
        return bb

    @staticmethod
    def text2latex(s):
        # Converts user text to LaTeX text
        if s is None:
            return None
        s = s.replace("\\", "\\backslash")
        s = s.replace("#", "\\#")
        s = s.replace("$", "\\$")
        s = s.replace("%", "\\%")
        s = s.replace("^", "\\^{}")
        s = s.replace("&", "\\&")
        s = s.replace("_", "\\_")
        s = s.replace("{", "\\{")
        s = s.replace("}", "\\}")
        s = s.replace("~", "\\~{}")
        s = s.replace("\xc3\x9f", "{\ss}") # replace ss
        s = s.replace("\xc3\xa4", "{\"a}") # replace ae
        s = s.replace("\xc3\x84", "{\"A}") # replace AE
        s = s.replace("\xc3\x9c", "{\"U}") # replace UE
        s = s.replace("\xc3\xbc", "{\"u}") # replace ue
        s = s.replace("\xc3\x96", "{\"O}") # replace OE
        s = s.replace("\xc3\xb6", "{\"o}") # replace oe
        return s

    def generate_latex_shift_header(self, latexf, shift):
        print(" - Generating the Latex section for shift with id " + str(shift["id"]))
        # Chapter title.
        latexf.write('\\section{Shift %s: %s -- %s (%s)}\n\n' % (shift["id"], shift['shift_start_datetime'],
                                                                 shift['shift_end_datetime'],
                                                                 shift['performer_institution']))
        # Overview map.
        latexf.write('\\subsection{Overview map}\n\n')
        image_path_file = "{}/{}/shift_{}.jpeg".format(builddir, imagedir, shift["id"])
        latex_image_path_file = "{}/shift_{}.jpeg".format(imagedir, shift["id"])
        if os.path.isfile(image_path_file):
            latexf.write('The following map shows an overview of the whole shift.\n\n')
            latexf.write('\\vspace{0.5cm}\n')
            latexf.write('\\begin{center}\n\n')
            latexf.write('\\includegraphics[width=\maxwidth{16cm},height=15cm,keepaspectratio]{%s}\n\n' % latex_image_path_file)
            latexf.write('\\end{center}\n\n')
        else:
            latexf.write('(No overview map file available.)')
        # Shift summary.
        template = self.env.get_template(DEFAULT_SHIFT_HEADER_TEMPLATE)
        latexf.write(
            template.render(shiftStartDatetime=shift['shift_start_datetime'],
                            shiftEndDatetime=shift['shift_end_datetime'], id=shift['id'],
                            admin=shift['test_administrator_name'], director=shift['test_director_name'],
                            saftey=shift['safety_officer_name'], performer=shift['performer_institution'],
                            testIntent=shift['test_intent'], note=self.text2latex(shift['shift_note'])))

        return

    def generate_latex_shift_statistic(self, latexf, statistic):
        # Write ITO/AUTO statistics table.
        template = self.env.get_template(DEFAULT_SHIFT_STATISTICS_HEADER_TEMPLATE)
        latexf.write(
            template.render())
        for row in statistic:
            speed_m_s = 0
            if row['distance'] is not None and row['duration'] > 0:
                speed_m_s = row['distance'] / row['duration']
            speed_km_h = speed_m_s * 3.6
            speed_mi_h = speed_m_s * 2.236936292
            tmp_dur = self.conv_time(row['duration'])
            template = self.env.get_template(DEFAULT_SHIFT_STATISTICS_DATA_TEMPLATE)
            latexf.write(
                template.render(segmentTypeShortDescription=row['segment_type_short_description'], Count=row['cnt'],
                                Duration=tmp_dur, Distance="{:8.2f}".format(row['distance']),
                                Speedm="{:3.1f}".format(speed_m_s), SpeedKm="{:3.1f}".format(speed_km_h),
                                SpeedMi="{:3.1f}".format(speed_mi_h)))
        latexf.write('\\end{longtable}\n')
        latexf.write('\n')

    def generate_latex_segment(self, latexf, shift_id, local):
        # Extract main info from master segments of selected shift.
        sel_stmt = "SELECT to_timestamp(segment.starttime_secs) as start_datetime, \
                   to_timestamp(segment.endtime_secs) as end_datetime, \
                   segment.orig_starttime_secs as orig_start_timestamp, \
                   segment_type.short_description as segment_type_short_description, \
                   segment.endtime_secs - segment.starttime_secs as duration, \
                   segment.distance as distance, \
                   segment.leg_id, segment.id as segment_id, \
                   RTRIM(weather.short_description) as weather_short_description, leg.number as leg_number \
                   FROM segment inner join leg on segment.leg_id = leg.id \
                   LEFT OUTER JOIN segment_type on segment.segment_type_id = segment_type.id \
                   LEFT OUTER JOIN weather on leg.weather_id = weather.id \
                   WHERE segment.leg_id = leg.id and leg.shift_id = %s and segment.parent_id is null \
                   ORDER BY segment.starttime_secs" % shift_id
        self.db_adapter.dictcursor.execute(sel_stmt)
        segment_values = self.db_adapter.dictcursor.fetchall()
        # Write individual segment data to tex file.
        for row in segment_values:
            distance = self.get_segment_distance(row['segment_id'], row['leg_id'], local)
            # Get info from sub-segments for this master segment.
            sel_stmt = "SELECT ito_reason.short_description, segment.obstacle, segment.lighting, segment.slope " \
                       "FROM segment " \
                       "LEFT OUTER JOIN ito_reason on segment.ito_reason_id = ito_reason.id " \
                       "WHERE segment.id = %s or segment.parent_id = %s " \
                       "ORDER BY segment.starttime_secs" % (row['segment_id'], row['segment_id'])
            self.db_adapter.dictcursor.execute(sel_stmt)
            data = self.db_adapter.dictcursor.fetchall()
            # Format variables to write to tex template.
            weather = row['weather_short_description']
            if weather is None:
                weather = "--"
            duration = row['duration']
            if duration is None:
                duration = 0
            ito_reason = ""
            obstacle = ""
            lighting = ""
            slope = ""
            for row2 in data:
                if row2['short_description'] is not None:
                    if ito_reason:
                        ito_reason += ', '
                    ito_reason += '%s' % (row2['short_description'])
                if row2['obstacle'] is not None:
                    if obstacle:
                        obstacle += ', '
                    obstacle += '%s' % (row2['obstacle'])
                if row2['lighting'] is not None:
                    if lighting:
                        lighting += ', '
                    lighting += '%s' % (row2['lighting'])
                if row2['slope'] is not None:
                    if slope:
                        slope += ', '
                    slope += '%s' % (row2['slope'])
            if not ito_reason:
                ito_reason = '--'
            if not obstacle:
                obstacle = '--'
            if not lighting:
                lighting = '--'
            if not slope:
                slope = '--'
            # Get all notes for this master segment and its children (sub-segments).
            sel_stmt = "SELECT p.name, note.note " \
                       "FROM note INNER JOIN segment ON segment.id = note.segment_id " \
                       "LEFT OUTER JOIN personnel as p on note.personnel_id = p.id " \
                       "WHERE note.segment_id = %s or " \
                       "(note.segment_id = segment.id and segment.parent_id = %s)" % (row['segment_id'], row['segment_id'])
            self.db_adapter.dictcursor.execute(sel_stmt)
            data = self.db_adapter.dictcursor.fetchall()
            # Format notes to write to tex template.
            notes = ""
            for row2 in data:
                if notes:
                    notes += '; '
                notes += '%s: %s' % (self.text2latex(row2['name']), self.text2latex(row2['note']))
            if not notes:
                notes = '--'
            # Write variables to template and tex file.
            template = self.env.get_template(DEFAULT_SEGMENT_HEADER_TEMPLATE)
            latexf.write(
                template.render(segmentTypeShortDescription = row['segment_type_short_description'],
                                startDatetime = row['start_datetime'].strftime('%Y-%m-%d %H:%M:%S'),
                                origStartTimestamp = row['orig_start_timestamp'],
                                origStartDatetime = datetime.datetime.fromtimestamp(int(row['orig_start_timestamp'])).strftime('%Y-%m-%d %H:%M:%S') if row['orig_start_timestamp'] else "",
                                segmentId = row['segment_id'],
                                legNumber = row['leg_number'],
                                endDatetime = row['end_datetime'].strftime("%Y-%m-%d %H:%M:%S"),
                                duration = "{:8.2f}".format(duration), 
                                distance = "{:8.2f}".format(distance), 
                                obstacle = obstacle, 
                                lighting = lighting,
                                slope = slope, 
                                weather = weather,
                                itoReason = ito_reason,
                                notes = notes
                )
            )
            # Get bounding box for the segment location points.
            sel_stmt = "SELECT st_extent(pose.position) as seg_bb \
                       FROM %s AS pose \
                       INNER JOIN segment ON pose.segment_id = segment.id \
                       WHERE segment_id = %s" % ("local_pose" if local else "pose", row['segment_id'])
            self.db_adapter.dictcursor.execute(sel_stmt)
            data = self.db_adapter.dictcursor.fetchone()
            bb_str = data[0]
            # Build map image.
            if bb_str is not None:
                seg_bb = self.get_bb_from_text(bb_str)
                if local:
                    self.get_local_map(shift_id, row['segment_id'], seg_bb)
                else:
                    self.get_map(shift_id, row['segment_id'], seg_bb)
            # Attach map image to tex file.
            image_path_file = "%s/%s/segment_%s.jpeg" % (builddir, imagedir, row['segment_id'])
            if os.path.isfile(image_path_file):
                latexf.write('\\vspace{0.2cm}\n')
                latexf.write('\\begin{center}\n\n')
                latexf.write('\\includegraphics[width=\maxwidth{16cm},height=15cm,keepaspectratio]{%s/segment_%s.jpeg}\n\n' % (imagedir, row['segment_id']))
                latexf.write('\\end{center}\n\n')
            else:
                latexf.write('(No map file available.)')
            latexf.write('\\newpage\n\n')
            # List all images for this master segment and its children
            sel_stmt = "SELECT image.image_data as image, image.image_filename as filename, RTRIM(image.description) " \
                       "as description, image.segment_id as image_segment_id, " \
                       "segment.parent_id as segment_parent_id " \
                       "FROM image, segment " \
                       "WHERE (segment.id = %s and segment.id = image.segment_id) or (segment.parent_id = %s " \
                       "and segment.id = image.segment_id)" \
                       "ORDER BY image.id" % (row['segment_id'], row['segment_id'])

            self.db_adapter.dictcursor.execute(sel_stmt)
            data = self.db_adapter.dictcursor.fetchall()
            # Attach images and write description text to tex file.
            for count, row2 in enumerate(data):
                jpg_image_data = base64.standard_b64decode(row2['image'])
                if not (jpg_image_data.startswith('Error 404: Not Found')):
                    tmp_file = "%s/%s/%s_%s_cam.jpeg" % (builddir, imagedir, row2['image_segment_id'], count)
                    f = open(tmp_file, 'w')
                    f.write(jpg_image_data)
                    f.close()
                    latexf.write('\\vspace{0.2cm}\n')
                    latexf.write('\\begin{center}\n\n')
                    latexf.write(
                        '\\includegraphics[width=12cm,height=10cm,keepaspectratio]{%s/%s_%s_cam.jpeg}\n\n' % (
                            imagedir, row2['image_segment_id'], count))
                    if row2['description'] is not None and row2['description'] != "":
                        latexf.write('%s.\\\\\n' % (row2['description'].replace("_", "\_")))
                    else:
                        latexf.write('Image captured from the chase vehicle.')
                    latexf.write('\\end{center}\n\n')
                else:
                    print("Warning: Image data starts with 'Error 404: Not Found'")
            # End page for this segment.
            latexf.write('\\newpage\n\n')
        # End page for this shift.
        latexf.write('\\newpage\n\n')

    @staticmethod
    def generate_latex_shift_timeline(latexf, shift, shift_timeline, min_dur):
        print("Shift timeline:")

        pattern = '%Y-%m-%d %H:%M:%S'
        shift_start_datetime = shift["shift_start_datetime"]
        shift_start_secs = int(time.mktime(time.strptime(shift_start_datetime, pattern)))
        print("shift_start_secs : %s" % (shift_start_secs,))
        shift_end_datetime = shift["shift_end_datetime"]
        shift_end_secs = int(time.mktime(time.strptime(shift_end_datetime, pattern)))
        print("shift_end_secs : %s" % (shift_end_secs,))

        # Make sure that the bar is at least n hours long
        t_min = float(min_dur) * 3600
        shift_duration_sec = shift_end_secs - shift_start_secs
        if shift_duration_sec < t_min:
            shift_duration_sec = t_min

        page_width_cm = 15.0
        cm_p_s = page_width_cm / shift_duration_sec
        latexf.write('\\subsection{Shift timeline}\n\n')
        latexf.write('\\begin{tikzpicture}[scale=1.0]\n')
        latexf.write('\\draw[->] (0,0) -- (%f,0);' % (shift_duration_sec * cm_p_s))
        max_lines = 16  # including the top lines AUTO / ITO
        start_flex_line = 5
        line = start_flex_line
        line_sep = -0.5
        t = 0
        last_weather_icon_filename = ""
        while t <= shift_duration_sec:
            abs_time = str(datetime.timedelta(seconds=t))
            if shift_duration_sec < 3600:
                abs_time = abs_time[3:] + 's' # [3:] to remove the hours
            else:
                abs_time = abs_time[:-3] + 'm' # [:-3] to remove the seconds
            latexf.write('\\draw (%f,-0.1) -- (%f,0) node[anchor=south] {%s};\n' % (t * cm_p_s, t * cm_p_s, abs_time))
            t = t + shift_duration_sec / 10
        for row in shift_timeline:
            print("============== New segment ============")
            segment_type_key = row['segment_type_key']
            if (row['starttime'] is not None) & (shift['shift_start_datetime_raw'] is not None):
                start_secs = (row['starttime'] - shift["shift_start_datetime_raw"]).total_seconds()
            elif row['starttime'] is None:
                start_secs = 0
                print("Error Startitme is %s" % row['starttime'])
            else:
                start_secs = 0
                print("Error Datetime Raw is %s" % shift['shift_start_datetime_raw'])

            print("start_datetime. %s" % (row['starttime'],))
            print("start_datetime from shift. %s" % (start_secs,))
            if row['duration'] is not None:
                duration = row["duration"].total_seconds()
                print("duration: %s" % (duration,))
            else:
                print("Error Duration is unavible")

            segment_id = row['segment_id']
            segment_parent_id = row['segment_parent_id']
            segment_color = row['segment_color']
            parent_id = row['parent_id']
            ito_key = row['ito_key']
            ito_color = row['ito_color']
            ito_description = row['ito_description']
            if ito_description is None:
                ito_description = "--"
            if ito_description == "":
                ito_description = "--"
            weather_icon_filename = row['weather_icon_filename']
            is_master_seg = (parent_id is None or parent_id == '')
            if is_master_seg:
                jump_label = 'segment_%s' % segment_id
            else:
                jump_label = 'segment_%s' % segment_parent_id
            # determine y position and color
            if duration is None:
                duration = 0
            # print "Duration: ", duration
            if (duration * cm_p_s) > 0 or is_master_seg:
                # with duration or bar in upper line
                if is_master_seg:
                    y = 0
                    # Add weather icon
                    if weather_icon_filename is None:
                        weather_icon_filename = "unknown_weather"
                    if last_weather_icon_filename != weather_icon_filename:
                        latexf.write("\\pgftext[left, x=%fcm,y=%fcm] {\includegraphics[width=0.7cm]{%s/%s}}" % (
                            start_secs * cm_p_s, 1 * line_sep, logosdir, weather_icon_filename))
                        last_weather_icon_filename = weather_icon_filename
                    # master segment in upper rows
                    ito_description = ''
                    color = segment_color
                    if segment_type_key == 'auto':
                        y = 3 * line_sep
                    if segment_type_key == 'ito':
                        y = 4 * line_sep
                else:
                    # child segment in lower rows
                    color = ito_color
                    y = line * line_sep
                    line = line + 1
                    latexf.write('\\draw[dashed, color=lightgray] (%f,0) -- (%f,%f);\n' % (
                        start_secs * cm_p_s, start_secs * cm_p_s, y))
                latexf.write('\\definecolor{currentcolor}{RGB}{%s}\n' % color)
                latexf.write(
                    '\\draw[fill=currentcolor] (%f,%f-0.125) rectangle +(%f,0.25) node[anchor=west,yshift=-3pt]{'
                    '\\hyperref[%s]{%s}};\n' %
                    (start_secs * cm_p_s, y, duration * cm_p_s, jump_label, ito_description))
            else:
                # Segment without duration
                latexf.write('\\definecolor{currentcolor}{RGB}{%s}\n' % ito_color)
                if ito_key == 'goal' or ito_key == 'end_shift':
                    y = 2 * line_sep
                    latexf.write(
                        '\\draw[fill=currentcolor] (%f,%f) circle (.1cm) node[anchor=west,xshift=0.1cm,fill=white] {'
                        '\\hyperref[%s]{%s}};\n' %
                        (start_secs * cm_p_s, y, jump_label, ito_description))
                else:
                    y = line * line_sep
                    latexf.write(
                        '\\draw[fill=currentcolor] (%f,%f) circle (.1cm) node[anchor=west,xshift=1pt] {\\hyperref['
                        '%s]{%s}};\n' %
                        (start_secs * cm_p_s, y, jump_label, ito_description))
                    latexf.write('\\draw[dashed, color=lightgray] (%f,0) -- (%f,%f);\n' %
                                 (start_secs * cm_p_s, start_secs * cm_p_s, y))
                    line = line + 1
            if line > max_lines:
                line = start_flex_line
        latexf.write('\\draw[->] (0,%f) -- (%f,%f);' % (
            (max_lines + 1) * line_sep, shift_duration_sec * cm_p_s, (max_lines + 1) * line_sep))
        t = 0
        while t <= shift_duration_sec:
            abs_time = t + int(shift_start_secs)
            abs_time_str = datetime.datetime.utcfromtimestamp(abs_time).strftime('%H:%M')
            latexf.write('\\draw (%f,0) -- (%f,%f) node[anchor=north] {%s};\n' % (
                t * cm_p_s, t * cm_p_s, (max_lines + 1) * line_sep - 0.1, abs_time_str))
            t = t + 900  # 900 sec = 15 min
        latexf.write('\\end{tikzpicture}\n')
        latexf.write('\\newpage\n\n')

    def generate_latex_header(self, latexf, report_info):
        print(">>> Generating the Latex files for " + report_info["test_event_name"])
        template = self.env.get_template(report_info["latex_template"])
        latexf.write(template.render(
            latexTestCampaignName = report_info["test_event_name"], 
            version = report_info["report_version"],
            recipientName = report_info["recipient_name"],
            recipientAddress1 = report_info["recipient_address_l1"],
            recipientAddress2 = report_info["recipient_address_l2"],
            creatorName = report_info["creator_name"],
            creatorAddress1 = report_info["creator_address_l1"],
            creatorAddress2 = report_info["creator_address_l2"],
            topLogoPath = report_info["top_logo_path"],
            bottomLogoPath = report_info["bottom_logo_path"]
        ))
        return latexf

    @staticmethod
    def generate_latex_end(latexf):
        latexf.write('''\\end{document}''')
        latexf.close()

    def generate_latex_report(self, report_info):
        # Create the Filepath + Name of the .tex-File which is to b written
        filename = '%s/%s.tex' % (builddir, report_info["test_event_filename"])
        # Opens the File in write Mode
        latex_f = open(filename, 'w')
        print("Writing report %s" % (filename,))
        # Generates the Latex Header
        self.generate_latex_header(latex_f, report_info)
        # Reads the Shift Values and Id's out of the DB
        shifts = self.get_shifts_by_test_event_id(report_info["test_event_id"], report_info["local"])

        # Iterate over each Shift
        for shift in shifts:
            # With shift Id
            shift_id = shift["id"]
            # Start time of the selected Shift
            shift_start_datetime = shift["shift_start_datetime"]
            # End time of the selectedShift
            shift_end_datetime = shift["shift_end_datetime"]
            # Boundingbox of the Shift text
            bb_str = shift["shift_bb"]
            print("============== New Shift %s ============" % shift_id)
            print("Adding Shift %s (%s, %s)" % (shift_id, shift_start_datetime, shift_end_datetime))

            if not (bb_str is None):
                bb = self.get_bb_from_text(bb_str)
                # Gets the Map and will save it to ~/build/images/
                if report_info["local"]:
                    # Get map from local position data.
                    self.get_local_map(shift_id, None, bb)
                else:
                    # Get map from GPS data.
                    self.get_map(shift_id, None, bb)
            # Generates the Latex Shift Header
            self.generate_latex_shift_header(latex_f, shift)
            # Select the Shift statistics from the DB
            shift_statisics = self.get_shift_statistics(shift_id, report_info["local"])
            # Generate the Shift statistics for a Latex file
            self.generate_latex_shift_statistic(latex_f, shift_statisics)
            # List the segments of the shift
            shift_timeline = self.get_shift_timeline(shift_id)
            # Create the Timeline for the Shift
            self.generate_latex_shift_timeline(latex_f, shift, shift_timeline, report_info['min_duration'])
            self.generate_latex_segment(latex_f, shift_id, report_info["local"])

        self.generate_latex_end(latex_f)
        return

class ReportGenerator:

    def init_dirs(self):
        img_dir_str = "%s/%s" % (builddir, imagedir)
        map_dir_str = "%s/%s" % (builddir, maptiledir)
        if not os.path.exists(builddir):
            os.makedirs(builddir)
        if not os.path.exists(img_dir_str):
            os.makedirs(img_dir_str)
        if not os.path.exists(map_dir_str):
            os.makedirs(map_dir_str)
        return


    def read_xml(self, filename):
        # Initialize return objects
        report_info_list = []
        db_info = {}
        # Open and parse XML.
        with open(filename, 'rb') as xml_file:
            root = etree.parse(xml_file).getroot()
        # Retrieve data from XML.
        for child in root:
            # Get tile server name.
            if child.tag == "resource":
                global TILE_SERVER
                global ZOOM_LEVEL
                TILE_SERVER = child.get("tile_server")
                try:
                    ZOOM_LEVEL = int(child.get("zoom_level"))
                except ValueError:
                    pass
            # Get technical db info.
            if child.tag == "postgis":
                db_info['dbname'] = child.get("dbname")
                db_info['host'] = child.get("host")
                db_info['user'] = child.get("user")
                db_info['password'] = child.get("password")
            # Get report info.
            if child.tag == "report":
                report_info = {}
                report_info['test_event_name'] = child.get("name")
                report_info['report_version'] = child.get("version")
                report_info['test_event_filename'] = "report" #report_info['test_event_name'].lower().replace(" ", "_")
                for k in child:
                    if k.tag == "test_event":
                        report_info['test_event_id'] = k.get("id")
                        report_info['min_duration'] = k.get("min_dur")
                        report_info['local'] = k.get("local").lower() == "true"
                    if k.tag == "recipient":
                        report_info['recipient_name'] = k.get("name")
                        report_info['recipient_address_l1'] = k.get("address_l1")
                        report_info['recipient_address_l2'] = k.get("address_l2")
                    if k.tag == "creator":
                        report_info['creator_name'] = k.get("name")
                        report_info['creator_address_l1'] = k.get("address_l1")
                        report_info['creator_address_l2'] = k.get("address_l2")
                    if k.tag == "logos":
                        report_info['top_logo_path'] = k.get("top_logo_path")
                        report_info['bottom_logo_path'] = k.get("bottom_logo_path")
                report_info_list.append(report_info)

        return report_info_list, db_info

    def generate(self, argv):
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        self.init_dirs()
        # Parse the XML-File and save the Values to corresponding Vars
        (report_info_list, db_info) = self.read_xml(argv[0])

        # Read connection details from the XML-File-Var and connect to the DB
        ftt_adapter = PgAdapter(db_info['host'], db_info['dbname'], db_info['user'], db_info['password'])
        ftt_adapter.connect()

        # Init the ReportGenerator
        env = Environment(
            variable_start_string = '\VAR{',
	        variable_end_string = '}',
            loader=FileSystemLoader(dname+'/templates/')
        )
        ftt_report_generator = FttReportGenerator(ftt_adapter, env)

        for report_info in report_info_list:
            report_info["latex_template"] = DEFAULT_HEADER_LATEX_TEMPLATE
            ftt_report_generator.generate_latex_report(report_info)
            call_str = '%s.tex' % (report_info['test_event_filename'])
            main_path = os.getcwd()
            os.chdir(builddir)
            call(['pdflatex', call_str])
            call(['pdflatex', call_str])
            file_path = os.path.normpath(report_info['test_event_filename'] + '.pdf')
            abs_file_path = os.path.abspath(file_path)
            call(['cp', abs_file_path, builddir])
            os.chdir(main_path)

if __name__ == "__main__":
    report_generator = ReportGenerator()
    report_generator.generate(sys.argv[1:])