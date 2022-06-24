#!/usr/bin/env python3
"""dbAdapter.py: classes to access the FTT database."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import psycopg2
import psycopg2.sql
import psycopg2.extras

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
        print(("Connecting to db: %s" % (self.dbname,)))
        self.conn = psycopg2.connect(conn_string)
        self.cursor = self.conn.cursor()
        self.dictcursor = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)

class FttAdapter(PgAdapter):
    def read_color_dict(self, tablename):
        color_dict = {}
        query = psycopg2.sql.SQL(
            "SELECT key, color \
            FROM {}"
        ).format(psycopg2.sql.Identifier(tablename))
        self.dictcursor.execute(query)
        for row in self.dictcursor.fetchall():
            color_dict[row['key']] = row['color']
        return color_dict

    def get_master_segment_ids(self, shift_id):
        # Get all master segment ids for the shift
        sel_stmt = "SELECT segment.id \
                    FROM segment \
                    INNER JOIN leg ON segment.leg_id = leg.id \
                    INNER JOIN shift on leg.shift_id = shift.id \
                    WHERE shift.id = %s AND segment.parent_id IS NULL \
                    ORDER BY segment.id"
        self.dictcursor.execute(sel_stmt, (shift_id,))
        return [x[0] for x in self.dictcursor.fetchall()]

    def get_master_segments(self, shift_id):
        # Extract main info from master segments of selected shift.
        sel_stmt = "SELECT to_timestamp(segment.starttime_secs) as start_datetime, \
                        to_timestamp(segment.endtime_secs) as end_datetime, \
                        segment.orig_starttime_secs as orig_start_timestamp, \
                        segment_type.short_description as segment_type_short_description, \
                        segment.endtime_secs - segment.starttime_secs as duration, \
                        segment.distance as distance, \
                        segment.leg_id, \
                        segment.id as segment_id, \
                        RTRIM(weather.short_description) as weather_short_description, \
                        leg.number as leg_number \
                   FROM segment \
                   INNER JOIN leg on segment.leg_id = leg.id \
                   LEFT OUTER JOIN segment_type on segment.segment_type_id = segment_type.id \
                   LEFT OUTER JOIN weather on leg.weather_id = weather.id \
                   WHERE segment.leg_id = leg.id and leg.shift_id = %s and segment.parent_id is null \
                   ORDER BY segment.id"
        self.dictcursor.execute(sel_stmt, (shift_id,))
        return self.dictcursor.fetchall()

    def get_sub_segments(self, segment_id):
        # Get info from sub-segments for this master segment.
        sel_stmt = "SELECT ito_reason.short_description, segment.obstacle, segment.lighting, segment.slope \
                    FROM segment \
                    LEFT OUTER JOIN ito_reason on segment.ito_reason_id = ito_reason.id \
                    WHERE segment.id = %s or segment.parent_id = %s \
                    ORDER BY segment.starttime_secs"
        self.dictcursor.execute(sel_stmt, (segment_id, segment_id))
        return self.dictcursor.fetchall()
            
    def get_segment_notes(self, segment_id):
        # Get all notes for this master segment and its children (sub-segments).
        sel_stmt = "SELECT p.name, note.note \
                    FROM note \
                    INNER JOIN segment ON segment.id = note.segment_id \
                    LEFT OUTER JOIN personnel as p on note.personnel_id = p.id \
                    WHERE note.segment_id = %s \
                        OR (note.segment_id = segment.id and segment.parent_id = %s) \
                    ORDER BY note.secs"
        self.dictcursor.execute(sel_stmt, (segment_id, segment_id))
        return self.dictcursor.fetchall()
            
    def get_segment_images(self, segment_id):
        # List all images for this master segment and its children
        sel_stmt = "SELECT image.image_data as image, \
                        image.image_filename as filename, \
                        RTRIM(image.description) as description, \
                        image.segment_id as image_segment_id, \
                        segment.parent_id as segment_parent_id \
                    FROM image, segment \
                    WHERE (segment.id = %s and segment.id = image.segment_id) OR (segment.parent_id = %s \
                        AND segment.id = image.segment_id) \
                    ORDER BY image.id"
        self.dictcursor.execute(sel_stmt, (segment_id, segment_id))
        return self.dictcursor.fetchall()

    def get_segment_bb(self, segment_id, local=False):
        # Get bounding box for the segment location points.
        sel_stmt = psycopg2.sql.SQL(
                    "SELECT st_extent(pose.position) as seg_bb \
                    FROM {} AS pose \
                    INNER JOIN segment ON pose.segment_id = segment.id \
                    WHERE segment_id = %s"
        ).format(psycopg2.sql.Identifier("local_pose" if local else "pose"))

        self.dictcursor.execute(sel_stmt, (segment_id,))
        return self.dictcursor.fetchone()

    def get_segment_border_points_and_color(self, seg_id, local=False):
        # Get this and next segment's starting point and this segment type color from database.
        if local:
            # Local positions
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
                        LIMIT 1"
        else:
            # In web mercator projection
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
                        LIMIT 1"

        self.dictcursor.execute(sel_stmt, (seg_id,))
        return self.dictcursor.fetchone()

    def get_segment_points(self, seg_id, local=False):
        # Get segment points from database
        if local:
            # Local Positions
            sel_stmt = "SELECT ST_X(local_pose.position), ST_Y(local_pose.position) \
                        FROM local_pose \
                        INNER JOIN segment ON local_pose.segment_id = segment.id \
                        WHERE segment_id = %s \
                        ORDER BY local_pose.id"
        else:
            # In web mercator projection.
            sel_stmt = "SELECT ST_X(ST_TRANSFORM(pose.position, 3857)), ST_Y(ST_TRANSFORM(pose.position, 3857)) \
                        FROM pose \
                        INNER JOIN segment ON pose.segment_id = segment.id \
                        WHERE segment_id = %s \
                        ORDER BY pose.id"
        
        self.dictcursor.execute(sel_stmt, (seg_id,))
        return self.dictcursor.fetchall()

    def get_segment_distance(self, seg_id, local):
        # Get distance between segment start position and first location point.
        if local:
            select_stmt = "SELECT st_distance (segment.local_start_position, local_pose.position) \
                        FROM segment \
                        INNER JOIN local_pose ON local_pose.segment_id = segment.id \
                        WHERE segment.id = %s \
                        ORDER BY local_pose.secs \
                        LIMIT 1"
        else:  
            select_stmt = "SELECT st_distance (st_setsrid(segment.start_position,4326), st_setsrid(pose.position,4326), true) \
                        FROM segment \
                        INNER JOIN pose ON pose.segment_id = segment.id \
                        WHERE segment.id = %s \
                        ORDER BY pose.secs \
                        LIMIT 1"
        self.dictcursor.execute(select_stmt, (seg_id,))
        data = self.dictcursor.fetchone()
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
                        AND this.segment_id = %s"
        else:
            select_stmt = "SELECT \
                    SUM (st_distance (st_setsrid(this.position,4326), st_setsrid(next.position,4326), true)) \
                    FROM pose as this, pose as next \
                    WHERE this.id+1 = next.id  \
                        AND this.segment_id = next.segment_id \
                        AND this.segment_id = %s"
        self.dictcursor.execute(select_stmt, (seg_id,))
        data = self.dictcursor.fetchone()
        if data and data[0]:
            distance = data[0]
        else:
            distance = 0
        # Get distance between last location point and next segment's start position.
        #  - First get the segment's leg_id
        select_stmt = "SELECT leg_id \
            FROM segment \
            WHERE segment.id = %s"
        self.dictcursor.execute(select_stmt, (seg_id,))
        data = self.dictcursor.fetchone()
        leg_id = data[0]
        # - Then find and get the distance to the next segment
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
                        WHERE segment_id = %s)"
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
                        WHERE segment_id = %s)"
        self.dictcursor.execute(select_stmt, (seg_id, leg_id, seg_id))
        data = self.dictcursor.fetchone()
        if data and data[0]:
            end_distance = data[0]
        else:
            end_distance = 0
        # Add distances.
        distance = start_distance + distance + end_distance
        # Store the distance entry in the segment table.
        update_stmt = "UPDATE segment \
                        SET distance = %s \
                        WHERE id = %s"
        self.dictcursor.execute(update_stmt, (distance, seg_id))
        self.conn.commit()
        return distance

    def get_segment_duration(self, seg_id):
        select_stmt = "SELECT endtime_secs - starttime_secs \
            FROM segment \
            WHERE segment.id = %s"
        self.dictcursor.execute(select_stmt, (seg_id,))
        data = self.dictcursor.fetchone()
        if data and data[0]:
            duration = data[0]
        else:
            duration = 0
        return duration

    def get_local_map(self, shift_id):
        # Get the local map image from the database.
        sel_stmt = "SELECT resolution, ST_X(origin) as origin_x, ST_Y(origin) as origin_y, image_data as image \
            FROM map_image \
            WHERE shift_id = %s"
        self.dictcursor.execute(sel_stmt, (shift_id,))
        return self.dictcursor.fetchone()

    def get_shifts_by_test_event_id(self, test_event_id, local):
        select_stmt = psycopg2.sql.SQL(
            "SELECT shift.id as id, \
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
            LEFT OUTER JOIN personnel as user_test_administrator on shift.test_administrator_id = user_test_administrator.id \
            LEFT OUTER JOIN personnel as user_test_director on shift.test_director_id = user_test_director.id \
            LEFT OUTER JOIN personnel as user_safety_officer on shift.safety_officer_id = user_safety_officer.id \
            LEFT OUTER JOIN performer on shift.performer_id = performer.id \
            WHERE test_event_id = %s \
            GROUP BY shift.id, test_administrator_name, test_director_name, safety_officer_name, performer_institution \
            ORDER BY shift.id"
        ).format(psycopg2.sql.Identifier("local_pose" if local else "pose"))
        # NOTE: The retrieved bounding box does not consider the segment border points (segment start position)
        self.dictcursor.execute(select_stmt, (test_event_id,))
        shifts = self.dictcursor.fetchall()
        return shifts

    def get_shift_timeline(self, shift_id):
        select_stmt = "SELECT segment.id as segment_id, \
                segment.parent_id as segment_parent_id, \
                segment_type.key as segment_type_key, \
                segment_type.short_description as segment_type_short_description, \
                segment_type.color as segment_color, \
                to_timestamp(segment.starttime_secs) as starttime, \
                to_timestamp(segment.endtime_secs) as endtime, \
                to_timestamp(segment.endtime_secs) - to_timestamp(segment.starttime_secs) as duration, \
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
        self.dictcursor.execute(select_stmt, (shift_id,))
        shift_timeline = self.dictcursor.fetchall()
        return shift_timeline

    def get_shift_statistics(self, shift_id, local):
        # Get segment types.
        select_stmt = "SELECT id, short_description \
            FROM segment_type"
        self.dictcursor.execute(select_stmt)
        data = self.dictcursor.fetchall()
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
                    AND segment.parent_id IS NULL"
            self.dictcursor.execute(select_stmt, (shift_id, segment_type_id))
            data = self.dictcursor.fetchall()
            # Get id count.
            stats['cnt'] = len(data)
            # Get total segment distance.
            stats['distance'] = sum([self.get_segment_distance(row[0], local) for row in data])
            # Get total segment duration.
            stats['duration'] = sum([self.get_segment_duration(row[0]) for row in data])
            # Add segment type statistics to result.
            shift_statistics.append(stats)

        return shift_statistics

    def get_shift_weather_icons(self, shift_id):
        select_stmt = "SELECT weather.icon_filename as weather_icon_filename \
                    FROM leg \
                    LEFT OUTER JOIN weather on leg.weather_id = weather.id \
                    WHERE shift_id = %s \
                    ORDER BY starttime_secs"
        self.dictcursor.execute(select_stmt, (shift_id,))
        return self.dictcursor.fetchall()

    def get_shift_maintenance_time(self, shift_id):
        select_stmt = "SELECT sum(segment.endtime_secs-segment.starttime_secs) \
                    FROM segment \
                    INNER JOIN leg ON segment.leg_id = leg.id \
                    LEFT OUTER JOIN ito_reason ON segment.ito_reason_id = ito_reason.id \
                    WHERE ito_reason.key = 'maintenance' AND segment.leg_id = leg.id AND leg.shift_id = %s"
        self.dictcursor.execute(select_stmt, (shift_id,))
        return self.dictcursor.fetchone()[0]

    def get_shift_ito_time(self, shift_id):
        select_stmt = "SELECT sum(segment.endtime_secs-segment.starttime_secs) \
                    FROM segment \
                    INNER JOIN leg ON segment.leg_id = leg.id \
                    LEFT OUTER JOIN segment_type ON segment.segment_type_id = segment_type.id \
                    WHERE segment_type.key = 'ito' \
                        AND segment.leg_id = leg.id \
                        AND segment.parent_id IS NULL \
                        AND leg.shift_id = %s"
        self.dictcursor.execute(select_stmt, (shift_id,))
        return self.dictcursor.fetchone()[0]

    def get_shift_ito_count(self, shift_id, ito_key):
        select_stmt = "SELECT count (*) \
                    FROM segment \
                    INNER JOIN leg ON segment.leg_id = leg.id \
                    LEFT OUTER JOIN ito_reason on segment.ito_reason_id = ito_reason.id \
                    WHERE ito_reason.key = %s AND leg.shift_id = %s"
        self.dictcursor.execute(select_stmt, (ito_key, shift_id))
        return self.dictcursor.fetchone()[0]

    def get_test_event_ito_statistics(self, test_event_id):
        select_stmt = "SELECT ito_reason.key as ito_key, ito_reason.short_description as ito_reason, count(ito_reason.key) as cnt \
            FROM segment \
            INNER JOIN leg ON segment.leg_id = leg.id  \
            INNER JOIN shift ON leg.shift_id = shift.id  \
            LEFT OUTER JOIN segment_type on segment.segment_type_id = segment_type.id \
            LEFT OUTER JOIN ito_reason on segment.ito_reason_id = ito_reason.id \
            WHERE shift.test_event_id = %s AND segment_type.key = 'ito' AND segment.parent_id IS NOT NULL \
            GROUP BY ito_key, ito_reason \
            ORDER BY cnt DESC"
        self.dictcursor.execute(select_stmt, (test_event_id,))
        return self.dictcursor.fetchall()