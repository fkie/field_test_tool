#!/usr/bin/env python3
"""mapGenerator.py: class to retrieve the environment's map and draw the vehicle's trajectory."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import os
import sys
import base64
import requests
import pyproj
from PIL import Image, ImageDraw
from io import BytesIO

EQUATOR_PERIMETER = 40075016.68557849
TILE_FIX_SIZE = 1024

###############################################################

class MapGenerator:
    def __init__(self, db_adapter, tile_server, tile_zoom, image_dir, map_tile_dir):
        self.db_adapter = db_adapter
        self.tile_server = tile_server
        self.tile_zoom = tile_zoom
        self.image_dir = image_dir
        self.map_tile_dir = map_tile_dir
 
    @staticmethod
    def webmercator2tiles(easting, northing, zoom):
        # Transform web mercator's projection coordinates (in meters) to tile server's coordinates (with decimal points here).
        n = 2.0 ** zoom
        equator_perimeter = EQUATOR_PERIMETER
        xtile = (0.5 + easting / equator_perimeter) * n
        ytile = (0.5 - northing / equator_perimeter) * n
        return (xtile, ytile)

    @staticmethod
    def tiles2webmercator(xtile, ytile, zoom):
        # Transform tile server's coordinates to web mercator's projection (in meters).
        n = 2.0 ** zoom
        equator_perimeter = EQUATOR_PERIMETER
        easting = (xtile / n - 0.5) * equator_perimeter
        northing = (0.5 - ytile / n) * equator_perimeter
        return (easting, northing)

    @staticmethod
    def tile_px_scale(pixels_per_tile, zoom):
        # Get server's tile scale (in meters), according to web mercator's transform.
        n = 2.0 ** zoom
        equator_perimeter = EQUATOR_PERIMETER
        return pixels_per_tile * n / equator_perimeter

    def download_map_tiles(self, xmin, ymin, xmax, ymax, zoom):
        # Return if there's no tile server assigned.
        if not self.tile_server:
            print("No server specified to download map tiles.")
            return     
        # Request tiles from server.
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                # Construct filename string.
                filename = "%s/%s_%s_%s.png" % (self.map_tile_dir, zoom, xtile, ytile)
                if not os.path.isfile(filename):
                    try:
                        # Construct URL string.
                        url_format = r"http://{0}/{1}/{2}/{3}.png"
                        imgurl=url_format.format(self.tile_server, zoom, xtile, ytile)
                        print(("Opening: " + imgurl))
                        # Read response.
                        response = requests.get(imgurl, headers={"user-agent":"Custom user agent"})
                        imgstr = response.content
                        # Save to file.
                        f_image = open(filename, 'w')
                        f_image.write(imgstr)
                        f_image.close()
                    except:
                        e = sys.exc_info()[0] 
                        print(("Couldn't download tile %s/%s/%s" % (zoom, xtile, ytile)))
                        print(("Error: %s" % e))
        return

    def merge_map_tiles(self, xmin, ymin, xmax, ymax, zoom):
        # Create image canvas.
        map_images = Image.new('RGB',((xmax-xmin+1)*TILE_FIX_SIZE, (ymax-ymin+1)*TILE_FIX_SIZE))
        # Find downloaded map tiles and paste them in the canvas.
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                filename = "%s/%s_%s_%s.png" % (self.map_tile_dir, zoom, xtile, ytile)
                try:
                    # Get the tile.
                    tile = Image.open(filename)
                    # Fix the tile size.
                    tile = tile.resize((TILE_FIX_SIZE, TILE_FIX_SIZE), Image.ANTIALIAS)
                    # Paste it in the canvas.
                    map_images.paste(tile, box=((xtile-xmin)*TILE_FIX_SIZE ,  (ytile-ymin)*TILE_FIX_SIZE))
                except:
                    print(("Tile not found: %s/%s/%s" % (zoom, xtile, ytile)))
        return map_images

    def draw_segment_points(self, seg_id, map_images, xmin, ymin, zoom):
        # Define object for drawing over the map image.
        draw = ImageDraw.Draw(map_images)
        # Get this and next segment's starting point and this segment's color.
        data = self.db_adapter.get_segment_border_points_and_color(seg_id, False)
        start_easting = data[0]
        start_northing = data[1]
        end_easting = data[2]
        end_northing = data[3]
        color = tuple([int(i) for i in data[4].split(',')])
        # Get segment points.
        points = self.db_adapter.get_segment_points(seg_id, False)
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
        o_easting, o_northing = MapGenerator.tiles2webmercator(xmin, ymin, zoom)
        # Transform coordinates to image reference system.
        scale = MapGenerator.tile_px_scale(TILE_FIX_SIZE, zoom)
        seg_xy = [(scale * (point[0] - o_easting), scale * (o_northing - point[1])) for point in seg_xy]
        # Draw polyline on image.
        if len(seg_xy) > 1:
            draw.line(seg_xy, color, 3)
        # Draw a 9px wide point on each position.
        for p in seg_xy:
            draw.ellipse([p[0]-4, p[1]-4, p[0]+4, p[1]+4], color, color)

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
        zoom = self.tile_zoom
        # Find the coordinates in the tile's reference system (with decimal points).
        xmin_f, ymax_f = MapGenerator.webmercator2tiles(easting[0], northing[0], zoom)
        xmax_f, ymin_f = MapGenerator.webmercator2tiles(easting[1], northing[1], zoom)
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
        self.download_map_tiles(xmin, ymin, xmax, ymax, zoom)
        # Build the map image from the downloaded tiles.
        map_images = self.merge_map_tiles(xmin, ymin, xmax, ymax, zoom)
        # Add position data to image:
        if seg_id:
            # Draw segment points on image.
            self.draw_segment_points(seg_id, map_images, xmin, ymin, zoom)
        else:
            # Get all master segment ids for the shift
            segment_ids = self.db_adapter.get_master_segment_ids(shift_id)
            # Draw points on image for all the segments.
            for segment_id in segment_ids:
                self.draw_segment_points(segment_id, map_images, xmin, ymin, zoom)
        # Parametrize cropping to proportionally cut borders of the merged map image up to a minimum size of TILE_FIX_SIZE x TILE_FIX_SIZE px.
        h_crop_factor = max(0, min((map_images.width / TILE_FIX_SIZE - 1) / (xmin_excess + xmax_excess), 1))
        v_crop_factor = max(0, min((map_images.height / TILE_FIX_SIZE - 1) / (ymin_excess + ymax_excess), 1))
        # Crop excess size.
        map_images = map_images.crop((
            int(h_crop_factor * xmin_excess * TILE_FIX_SIZE), 
            int(v_crop_factor * ymin_excess * TILE_FIX_SIZE), 
            int(map_images.width - h_crop_factor * xmax_excess * TILE_FIX_SIZE), 
            int(map_images.height - v_crop_factor * ymax_excess * TILE_FIX_SIZE)
        ))
        # Define file name.
        if seg_id:
            file_dir = "%s/segment_%s.jpeg" % (self.image_dir, seg_id)
        else:
            file_dir = "%s/shift_%s.jpeg" % (self.image_dir, shift_id)
        # Save image to file.
        f_image = open(file_dir, 'w')
        map_images.save(f_image, "JPEG", dpi=(300, 300))
        f_image.close()
        return

    def draw_segment_local_points(self, seg_id, map_image, origin_x, origin_y, resolution):
        # Define object for drawing over the map image.
        draw = ImageDraw.Draw(map_image)
        # Get this and next segment's starting local point and this segment's color.
        data = self.db_adapter.get_segment_border_points_and_color(seg_id, True)
        start_easting = data[0]
        start_northing = data[1]
        end_easting = data[2]
        end_northing = data[3]
        color = tuple([int(i) for i in data[4].split(',')])
        # Get segment local points.
        points = self.db_adapter.get_segment_points(seg_id, True)
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
            draw.line(seg_xy, color, 3)
        # Draw a 9px wide point on each position.
        for p in seg_xy:
            draw.ellipse([p[0]-4, p[1]-4, p[0]+4, p[1]+4], color, color)

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
        map_data = self.db_adapter.get_local_map(shift_id)
        resolution = map_data["resolution"]
        origin_x = map_data["origin_x"]
        origin_y = map_data["origin_y"]
        map_image = Image.open(BytesIO(base64.standard_b64decode(map_data["image"]))).convert("RGB")
        # Make sure this image has a width of at least 4*TILE_FIX_SIZE
        map_image_width, map_image_height = map_image.size
        if map_image_width < 4*TILE_FIX_SIZE:
            resize_ratio = 4.0*TILE_FIX_SIZE/map_image_width
            # Fix the tile size.
            map_image = map_image.resize((int(map_image_width*resize_ratio), int(map_image_height*resize_ratio)), Image.ANTIALIAS)
            # Adjust the resolution accordingly.
            resolution /= resize_ratio
        # Add local position data to image:
        if seg_id:
            # Draw segment points on image.
            self.draw_segment_local_points(seg_id, map_image, origin_x, origin_y, resolution)
        else:
            # Get all master segment ids for the shift
            segment_ids = self.db_adapter.get_master_segment_ids(shift_id)
            # Draw points on image for all the segments.
            for segment_id in segment_ids:
                self.draw_segment_local_points(segment_id, map_image, origin_x, origin_y, resolution)
        # Get the distance between the original bounding box and the map image borders (in pixels).
        xmin_excess = (easting[0] - origin_x) / resolution # Distance between left edges.
        ymin_excess = (northing[0] - origin_y) / resolution # Distance between top edges.
        xmax_excess = map_image.width - (easting[1] - origin_x) / resolution # Distance between right edges.
        ymax_excess = map_image.height - (northing[1] - origin_y) / resolution # Distance between bottom edges.
        # Parametrize cropping to proportionally cut borders of the map image up to a minimum size of TILE_FIX_SIZE x TILE_FIX_SIZE px.
        h_crop_factor = max(0, min((map_image.width - TILE_FIX_SIZE) / (xmin_excess + xmax_excess), 1))
        v_crop_factor = max(0, min((map_image.height - TILE_FIX_SIZE) / (ymin_excess + ymax_excess), 1))
        # Crop excess size.
        map_image = map_image.crop((
            int(h_crop_factor * xmin_excess), 
            int(v_crop_factor * ymax_excess), 
            int(map_image.width - h_crop_factor * xmax_excess), 
            int(map_image.height - v_crop_factor * ymin_excess)
        ))
        # Define file name.
        if seg_id:
            file_dir = "%s/segment_%s.jpeg" % (self.image_dir, seg_id)
        else:
            file_dir = "%s/shift_%s.jpeg" % (self.image_dir, shift_id)
        # Save image to file.
        f_image = open(file_dir, 'w')
        map_image.save(f_image, "JPEG", dpi=(300, 300))
        f_image.close()
        return