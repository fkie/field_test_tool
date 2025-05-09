#!/usr/bin/env python3

"""api.py: Runs a web server to expose a JSON API to the FTT backend."""

__author__ = "Sebastian de la Cueva Couto Klein, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import os
import subprocess
import argparse, sys
import time
from datetime import datetime
from psycopg2 import connect, sql, IntegrityError
from flask import Flask, jsonify, request, send_from_directory, send_file
from flask_restful import Resource, Api
from flask_cors import CORS, cross_origin
from lxml import etree

app = Flask(__name__, static_url_path="", static_folder="../../ftt_web_interface")
CORS(app, support_credentials=True)
api = Api(app)

SEGMENT_TYPE_ITO = 1
SEGMENT_TYPE_AUTO = 2

'''
This RestfulAPI is used to communicate with our Fieldtest-Tool-Database via HTTP-commands.
Following commands are typically used:
    GET: Is used to request data from our database, e.g for filling a chart. 
    POST: Is used to send and create new data in our database.
    PUT: Is used to update existing data in our database.
    DELETE: Is used to delete existing data in our database (Not implemeted in this Code)
You can execute those by sending a request to their assigned endpoint. 

Here they are executing an SQL-statement which matches with their usage.
E.g. the GET-command excecutes a select-statement.
Each class has at least one of those commands.
You can find the corresponding endpoints at the bottom of the code.
'''

def print_and_return(print_string):
    # Global function. 
    # Prints the time and string argument then returns the argument.
    print(("["+datetime.now().strftime("%H:%M:%S")+"]"+print_string))
    return print_string

def info_msg(msg):
    return "[INFO]: %s" % msg

def warn_msg(msg):
    return "[WARN]: %s" % msg

def error_msg(error_type, msg):
    return "[ERROR] (%s): %s" % (error_type, msg)

class DBInterface:
    # Static class to wrap psycopg2 functions.

    @staticmethod
    def init(db_host, db_port):
        DBInterface.host = db_host or "localhost"
        DBInterface.port = db_port or "5432"
        DBInterface.conn = connect("host='%s' port='%s' dbname='ftt' user='postgres' password='postgres'" % (DBInterface.host, DBInterface.port))

    @staticmethod
    def execute(sql_stmt, params=None, returning=False, once=False):
        cursor = DBInterface.conn.cursor()
        try:
            cursor.execute(sql_stmt, params)
            result = None
            if returning and once:
                result = cursor.fetchone()
            elif returning and not once:
                result = cursor.fetchall()
            DBInterface.conn.commit()
            cursor.close()
            return result
        except Exception as err:
            DBInterface.conn.rollback()
            msg = error_msg(type(err).__name__, str(err))
            raise ApiError(msg, status_code=400)

    @staticmethod
    def close():
        DBInterface.conn.close()
        pass


class ApiError(Exception):
    # Class to raise server errors.
    status_code = 400

    def __init__(self, message, status_code=None, payload=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.payload = payload
        pass

    def to_dict(self):
        rv = dict(self.payload or ())
        rv["message"] = print_and_return(self.message)
        return rv

class ApiCommon:
    # Class to wrap common functionalities.

    @staticmethod
    def get_json():
        # Wrapper to get the json data from an http request.
        try:
            return request.get_json(force=True)
        except:
            msg = error_msg("DataError","Missing JSON data. DB query aborted.")
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def get_all_params_from_json(param_names):
        # Method to get a dictionary from json data with the values of 
        # ALL the required parameters from the requested arguments.
        json_data = ApiCommon.get_json()
        try:
            data = {}
            for param_name in param_names:
                data[param_name] = json_data[param_name]
            return data
        except KeyError as e:
            msg = error_msg("KeyError","Missing Key: '"+e[0]+"' In JSON data. DB query aborted.")
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def get_any_params_from_json(param_names):
         # Method to get a dictionary from json data with the values of 
         # AT LEAST ONE parameter from the requested arguments.
        json_data = ApiCommon.get_json()
        if any(param in json_data for param in param_names):
            data = {}
            for param_name in param_names:
                if param_name in json_data:
                    data[param_name] = json_data[param_name]
            return data
        else:
            msg = error_msg("KeyError","No matching parameters in JSON data. DB query aborted.")
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def get_all_params_from_url(param_names):
        # Method to get a dictionary from url data with the values of 
        # ALL the required parameters from the requested arguments.
        data = {}
        for param_name in param_names:
            value = request.args.get(param_name)
            if value is not None:
                data[param_name] = value
            else:
                msg = error_msg("KeyError","Missing Key: '"+param_name+"' In URL parameters. DB query aborted.")
                raise ApiError(msg, status_code=400)
        return data

    @staticmethod
    def get_any_params_from_url(param_names):
         # Method to get a dictionary from url data with the values of 
         # AT LEAST ONE parameter from the requested arguments.
        data = {}
        for param_name in param_names:
            value = request.args.get(param_name)
            if value is not None:
                data[param_name] = value
        if data:
            return data
        else:
            msg = error_msg("KeyError","No matching parameters in URL data. DB query aborted.")
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def get_all_params_from_url_or_json(param_names):
        data = None
        try:
            # Try to get data from URL.
            data = ApiCommon.get_all_params_from_url(param_names)
        except ApiError as error_url:
            try:
                # Try to get data from JSON data.
                data = ApiCommon.get_all_params_from_json(param_names)
            except ApiError as error_json:
                error_url_array = error_url.message.split("'")
                error_json_array = error_json.message.split("'")
                if len(error_json_array) > 1:
                    missing_key = error_json_array[1]
                elif len(error_url_array) > 1:
                    missing_key = error_url_array[1]
                msg = error_msg("KeyError","Missing Key: '"+missing_key+"' In URL or JSON parameters. DB query aborted.")
                raise ApiError(msg, status_code=400)
        return data

    @staticmethod
    def get_any_params_from_url_or_json(param_names):
        data = None
        try:
            # Try to get data from URL.
            data = ApiCommon.get_any_params_from_url(param_names)
        except ApiError as error:
            try:
                # Try to get data from JSON data.
                data = ApiCommon.get_any_params_from_json(param_names)
            except ApiError as error:
                msg = error_msg("KeyError","No matching parameters in URL or JSON data. DB query aborted.")
                raise ApiError(msg, status_code=400)
        return data

    @staticmethod
    def check_id(entry_id):
        # Method to check if the argument is a number.
        if not (isinstance(entry_id, int) or entry_id.isnumeric()):
            msg = error_msg("KeyValueError","Provided id is not a valid number. DB query aborted.")
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def get_sql_conditions(param_dict):
        # Method to get an array of psycopg sql objects for a condition statement with potential "None" values.
        # Each condition in the arry should have the from:
        # param_name = %s
        # OR
        # param_name IS %s (in the case the value is None)
        names = list(param_dict.keys())
        conditions = []
        for name in names:
            if param_dict[name] is not None:
                conditions.append(sql.SQL(' = ').join([sql.Identifier(name), sql.Placeholder()]))
            else:
                conditions.append(sql.SQL(' IS ').join([sql.Identifier(name), sql.Placeholder()]))
        return conditions

    @staticmethod
    def insert_table_data(table_name, param_dict):
        # SQL insert wrapper method to create a database entry in the specified table 
        # with the parameters from a dictionary passed in the arguments.
        names = list(param_dict.keys())
        values = list(param_dict.values())
        sql_stmt = sql.SQL(
            """
            INSERT INTO {} ({})
            VALUES ({})
            RETURNING id
            """
            ).format(
                sql.Identifier(table_name),
                sql.SQL(', ').join(map(sql.Identifier, names)),
                sql.SQL(', ').join(sql.Placeholder() * len(names))
            )
        entry_id = DBInterface.execute(sql_stmt, values, True, True)[0]
        return info_msg("Created %s %s entry." % (table_name, entry_id))

    @staticmethod
    def insert_table_data_with_position(table_name, param_dict, position_param_dict):
        # Extension of the insert_table_data method to insert postgis specific position data.
        names = list(param_dict.keys())
        position_names = list(position_param_dict.keys())
        values = list(param_dict.values())
        position_values = []
        for value in list(position_param_dict.values()):
            position_values.append(value["pos_x"])
            position_values.append(value["pos_y"])
            position_values.append(value["srid"])
        sql_stmt = sql.SQL(
            """
            INSERT INTO {} ({})
            VALUES ({}, %s)
            """ % ", ".join(["ST_GeomFromText('POINT(%s %s)', %s)"] * len(position_names))
            ).format(
                sql.Identifier(table_name),
                sql.SQL(', ').join(map(sql.Identifier, names + position_names)),
                sql.SQL(', ').join(sql.Placeholder() * len(names))
            )
        DBInterface.execute(sql_stmt, values + position_values)
        return info_msg("Created %s entry." % table_name)

    @staticmethod
    def update_table_data(table_name, param_dict, conditions):
        # SQL update wrapper ethod to update a database entry of the specified table 
        # with the parameters from a dictionary passed in the arguments.
        # Form query.
        sql_stmt = sql.SQL(
            """
            UPDATE {}
            SET {}
            WHERE {}
            """
            ).format(
                sql.Identifier(table_name),
                sql.SQL(', ').join((sql.SQL(' = ').join([s, sql.Placeholder()]) for s in map(sql.Identifier, list(param_dict.keys())))),
                sql.SQL(' AND ').join(ApiCommon.get_sql_conditions(conditions))
            )
        # Combine values from table columns and query conditions.
        sql_values = list(param_dict.values()) + list(conditions.values())
        # Execute query.
        DBInterface.execute(sql_stmt, sql_values)
        return info_msg("Updated %s table." % table_name)

    @staticmethod
    def update_table_data_with_position(table_name, param_dict, conditions, position_param_name, pos_x, pos_y, srid = 0):
        # Extension of the update_table_data method to update postgis specific position data.
        names = list(param_dict.keys())
        values = list(param_dict.values()) + [pos_x, pos_y, srid]
        # Form query.
        sql_stmt = sql.SQL(
            """
            UPDATE {}
            SET {}, {} = ST_GeomFromText('POINT(%s %s)', %s)
            WHERE {}
            """
            ).format(
                sql.Identifier(table_name),
                sql.SQL(', ').join((sql.SQL(' = ').join([s, sql.Placeholder()]) for s in map(sql.Identifier, names))),
                sql.Identifier(position_param_name),
                sql.SQL(' AND ').join(ApiCommon.get_sql_conditions(conditions))
            )
        # Combine values from table columns and query conditions.
        sql_values = values + list(conditions.values())
        # Execute query.
        DBInterface.execute(sql_stmt, sql_values)
        return info_msg("Updated %s table." % table_name)

    @staticmethod
    def get_table_data(table_name, param_names, conditions = None, order = None, limit = None):
        # SQL select wrapper method to get the database entries of the specified table.
        # The optional conditions parameters allows to specify a dictionary of conditions.
        # If the order parameter is set, the result is return ordered by id in ascending order, 
        # unless order = "DESC", in which case is returned in descending order.
        # Potential query values.
        sql_values = []
        # Base SQL statement.
        sql_stmt = sql.SQL(
            """
            SELECT {}
            FROM {}
            """
            ).format(
                sql.SQL(', ').join(map(sql.Identifier, param_names)),
                sql.Identifier(table_name)
            )
        if conditions:
            # Add condition values to query values.
            sql_values += list(conditions.values())
            # Add conditions to SQL query.
            sql_stmt += sql.SQL(
                """
                WHERE {}
                """
                ).format(
                    sql.SQL(' AND ').join(ApiCommon.get_sql_conditions(conditions))
                )
        if order:
            # Add result order to SQL query.
            sql_stmt += sql.SQL(
                """
                ORDER BY id%s
                """ % (" DESC" if order == "DESC" else "")
                )
        if limit:
            # Add limit value to query values.
            sql_values.append(limit)
            # Add limit to SQL query.
            sql_stmt += sql.SQL(
                """
                LIMIT %s
                """
                )
        if not conditions and not limit:
            sql_values = None
        if limit == 1:
            return DBInterface.execute(sql_stmt, sql_values, True, True)
        else:
            return DBInterface.execute(sql_stmt, sql_values, True, False)
    
    @staticmethod
    def delete_by_id(table_name):
        # Method to delete a database entry in the specified table 
        # with the id provided by the http request.
        # Get id from URL or JSON data.
        param_name = "id" 
        entry_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check if id is a valid number.
        ApiCommon.check_id(entry_id)
        sql_stmt = sql.SQL(
            """
            DELETE FROM {}
            WHERE id = %s;
            """
            ).format(sql.Identifier(table_name))
        DBInterface.execute(sql_stmt, (entry_id,))
        msg = info_msg("Deleted %s %s." % (table_name, entry_id))
        return print_and_return(msg)
    
    @staticmethod
    def get_open_log_ids(table_name, limit = None):
        # Specialization.
        # Method to get the ids of the "open" log entries of the specified table.
        entry = ApiCommon.get_table_data(table_name, ["id"], {"endtime_secs": None}, "DESC", limit)
        if entry:
            if limit == 1:
                return entry[0]
            else:
                return list([x[0] for x in entry])
        else:
            msg = error_msg("LogError","No open %s entry in the DB. Aborted." % table_name)
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def check_open_log(table_name, entry_id):
        # Specialization.
        # Method to check if the specified log entry is "open".
        try:
            open_ids = ApiCommon.get_open_log_ids(table_name)
            if int(entry_id) in open_ids: 
                return True
            else:
                raise ApiError()
        except:
            msg = error_msg("LogError", "Log entry %s %s is closed. Aborted." % (table_name, entry_id))
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def open_log(table_name, entry_id):
        # Specialization.
        # Method to open the specified log entry.
        ApiCommon.update_table_data(table_name, {"endtime_secs": None}, {"id": entry_id})
        msg = info_msg("Opened %s %s." % (table_name, entry_id))
        return print_and_return(msg)
    
    @staticmethod
    def close_log(table_name, entry_id):
        # Specialization.
        # Method to close the specified log entry.
        endtime_secs = int(time.time())
        ApiCommon.update_table_data(table_name, {"endtime_secs": endtime_secs}, {"id": entry_id})
        msg = info_msg("Closed %s %s." % (table_name, entry_id))
        return print_and_return(msg)

    @staticmethod
    def close_open_logs(table_name):
        # Specialization.
        # Method to close the "open" log entries of the specified table.
        endtime_secs = int(time.time())
        ApiCommon.update_table_data(table_name, {"endtime_secs": endtime_secs}, {"endtime_secs": None})
        msg = info_msg("Closed %s open entries." % table_name)
        return print_and_return(msg)

    @staticmethod
    def get_log_sibling_count(table_name, parent_id_name, parent_id):
        # Method to get the number of log entries with a common parent id.
        sql_stmt = sql.SQL(
        """
        SELECT COUNT({0})
        FROM {1}
        WHERE {0} = %s
        """
        ).format(
            sql.Identifier(parent_id_name),
            sql.Identifier(table_name),
        )
        row = DBInterface.execute(sql_stmt,(parent_id,), True, True)
        return row[0]

class TestEvent(Resource):

    @staticmethod
    def open(entry_id):
        # Close open log entries.
        TestEvent.close()
        # Open specified teset_event.
        table_name = "test_event"
        return ApiCommon.open_log(table_name, entry_id)
    
    @staticmethod
    def close(entry_id = None):
        table_name = "test_event"
        # If provided, check that the id is open.
        if entry_id:
            try:
                ApiCommon.check_open_log(table_name, entry_id)
            except ApiError as err:
                return err.message
        # Close open child log entry.
        Shift.close()
        # Close log entry.
        if entry_id:
            return ApiCommon.close_log(table_name, entry_id)
        else:
            return ApiCommon.close_open_logs(table_name)
        pass

    def get(self):
        # This method gets all test_event table data.
        col_names = ["id", "starttime_secs", "endtime_secs", "location", "version", "time_zone", "note"]
        return ApiCommon.get_table_data("test_event", col_names, None, True)  

    def post(self):
        # This method creates a new entry in the test event table.
        # Get parameter values.
        param_names = ["location", "version", "time_zone", "note"]
        data = ApiCommon.get_all_params_from_json(param_names)
        data["starttime_secs"] = int(time.time())
        # Close any open entries.
        TestEvent.close()
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("test_event", data))
        
        # Test from command line with:
        # curl -H "Content-Type: application/json" -d '{"test_event_location":"edsfs","test_event_text":"jabinichdfgh"}' http://localhost:5000/testevent

    def put(self):
        # This method updates an entry in the test event table.
        # Check if the request is to edit the data or close the log entry.
        param_names = ["id", "action"]
        data = ApiCommon.get_all_params_from_json(param_names)
        test_event_id = data[param_names[0]]
        action = data[param_names[1]]
        # Check that the id is a number.
        ApiCommon.check_id(test_event_id)
        if action == "edit":
            # Check for any properties to edit
            param_names = ["location", "version", "time_zone", "note"]
            data = ApiCommon.get_any_params_from_json(param_names)
            # Update table entry.
            return print_and_return(ApiCommon.update_table_data("test_event", data, {"id": test_event_id}))
        if action == "close":
            # Close table entry.
            return TestEvent.close(test_event_id)
        if action == "open":
            # Open table entry.
            return TestEvent.open(test_event_id)
        pass

    def delete(self):
        # This method deletes an entry in the test_event table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("test_event")


class Shift(Resource):

    @staticmethod
    def close(entry_id = None):
        table_name = "shift"
        # If provided, check that the id is open.
        if entry_id:
            try:
                ApiCommon.check_open_log(table_name, entry_id)
            except ApiError as err:
                return err.message
        # Close open child log entry.
        Leg.close()
        # Close log entry.
        if entry_id:
            return ApiCommon.close_log(table_name, entry_id)
        else:
            return ApiCommon.close_open_logs(table_name)
        pass   

    def get(self):
        # This method gets all shift table data.
        # Find test_event_id in url or json data.
        param_name = "test_event_id"
        test_event_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(test_event_id)
        # Return data.
        col_names = ["id", "test_event_id", "starttime_secs", "endtime_secs", "test_administrator_id", "test_director_id", "safety_officer_id", "robot_operator_id", "performer_id", "test_intent", "workspace", "vehicle_id", "note", "number"]
        return ApiCommon.get_table_data("shift", col_names, {"test_event_id": test_event_id}, True)

    def post(self):
        # This method creates a new entry in the shift table.
        # Get parameter values.
        param_names = ["test_event_id","test_administrator_id", "test_director_id", "safety_officer_id", "robot_operator_id", "performer_id", "test_intent", "workspace", "vehicle_id", "note"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Check that the requested test event is open.
        ApiCommon.check_open_log("test_event", data[param_names[0]])
        # Close any open entries.
        Shift.close()
        # Find shift count.
        number = ApiCommon.get_log_sibling_count("shift", "test_event_id", data[param_names[0]]) + 1
        # Complete post data.
        data["starttime_secs"] = int(time.time())
        data["number"] = number
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("shift", data))

    def put(self):
        # This method updates an entry in the shift table.
        # Check if the request is to edit the data or close the log entry.
        param_names = ["id", "action"]
        data = ApiCommon.get_all_params_from_json(param_names)
        shift_id = data[param_names[0]]
        action = data[param_names[1]]
        # Check that the id is a number.
        ApiCommon.check_id(shift_id)
        if action == "edit":
            # Check for any properties to edit
            param_names = ["test_administrator_id", "test_director_id", "safety_officer_id", "robot_operator_id", "performer_id", "test_intent", "workspace", "vehicle_id", "note"]
            data = ApiCommon.get_any_params_from_json(param_names)
            # Update table entry.
            return print_and_return(ApiCommon.update_table_data("shift", data, {"id": shift_id}))
        if action == "close":
            # Close table entry.
            return Shift.close(shift_id)
        pass

    def delete(self):
        # This method deletes an entry in the shift table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("shift")


class Leg(Resource):

    @staticmethod
    def close(entry_id = None):
        table_name = "leg"
        # If provided, check that the id is open.
        if entry_id:
            try:
                ApiCommon.check_open_log(table_name, entry_id)
            except ApiError as err:
                return err.message
        # Close open child log entries.
        Segment.close()
        # Close log entry.
        if entry_id:
            return ApiCommon.close_log(table_name, entry_id)
        else:
            return ApiCommon.close_open_logs(table_name)
        pass

    def get(self):
        # This method gets all leg table data.
        # Find shift id in url or json data.
        param_name = "shift_id"
        shift_id = ApiCommon.get_any_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(shift_id)
        # Return data.
        col_names = ["id", "shift_id", "starttime_secs", "endtime_secs", "weather_id", "default_pose_source_id", "note", "number"]
        return ApiCommon.get_table_data("leg", col_names, {"shift_id": shift_id}, True)

    def post(self):
        # This method creates a new entry in the leg table.
        # Get parameter values.
        param_names = ["shift_id", "weather_id", "default_pose_source_id", "note"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Check that the requested shift is open.
        ApiCommon.check_open_log("shift", data[param_names[0]])
        # Close any open entries.
        Leg.close()
        # Find leg count.
        number = ApiCommon.get_log_sibling_count("leg", "shift_id", data[param_names[0]]) + 1
        # Complete post data.
        data["starttime_secs"] = int(time.time())
        data["number"] = number
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("leg", data))

    def put(self):
        # This method updates an entry in the leg table.
        # Check if the request is to edit the data or close the log entry.
        param_names = ["id", "action"]
        data = ApiCommon.get_all_params_from_json(param_names)
        leg_id = data[param_names[0]]
        action = data[param_names[1]]
        # Check that the id is a number.
        ApiCommon.check_id(leg_id)
        if action == "edit":
            # Check for any properties to edit
            param_names = ["weather_id", "default_pose_source_id", "note"]
            data = ApiCommon.get_any_params_from_json(param_names)
            # Update table entry.
            return print_and_return(ApiCommon.update_table_data("leg", data, {"id": leg_id}))
        if action == "close":
            # Close table entry.
            return Leg.close(leg_id)
        pass

    def delete(self):
        # This method deletes an entry in the leg table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("leg")


class Segment(Resource):

    @staticmethod
    def close(entry_id = None):
        table_name = "segment"
        master_segment = False
        if entry_id:
            # If provided, check that the id is open.
            try:
                ApiCommon.check_open_log(table_name, entry_id)
            except ApiError as err:
                return err.message
            # Also check if the segment is a master segment
            try:
                Segment.check_master_segment(entry_id)
                master_segment = True
            except ApiError as err:
                master_segment = False
        if entry_id and not master_segment:
            # Close specified log entry if it's a child entry.
            return ApiCommon.close_log(table_name, entry_id)
        else:
            # Close all open entries for master segments and unspecified ids.
            return ApiCommon.close_open_logs(table_name)
        pass

    @staticmethod
    def get_current_master_segment(leg_id = None):
        # Get the open "master segment" (segment with no parent_id) id and type from the specified leg,
        # or the latest that is open.
        if leg_id is not None:
            entry = ApiCommon.get_table_data("segment", ["id", "segment_type_id"], {"leg_id": leg_id, "endtime_secs": None, "parent_id": None}, "DESC", 1)
        else:
            entry = ApiCommon.get_table_data("segment", ["id", "segment_type_id"], {"endtime_secs": None, "parent_id": None}, "DESC", 1)
        if entry is not None: 
            return entry[0], entry[1]
        else:
            msg = error_msg("LogError","No open master segment entry for %s. DB query aborted." % ("leg"+str(leg_id) if leg_id else "current leg"))
            raise ApiError(msg, status_code=400)
        pass

    @staticmethod
    def check_master_segment(segment_id):
        # Returns if segment_id references a segment with no parent_id, throws an ApiError otherwise.
        data = ApiCommon.get_table_data("segment", ["parent_id"], {"id": segment_id}, None, 1)
        if data is not None:
            if data[0] is None:
                return
            else:
                msg = error_msg("UnsupportedError","Segment %s is not a master. DB query aborted." % segment_id)
                raise ApiError(msg, status_code=400)
        else:
            msg = error_msg("UnsupportedError","Segment %s doesn't exist. DB query aborted." % segment_id)
            raise ApiError(msg, status_code=400)
        pass
    
    @staticmethod
    def check_segment_type(segment_type_id):
        if segment_type_id != SEGMENT_TYPE_ITO and segment_type_id != SEGMENT_TYPE_AUTO:
            msg = error_msg("UnsupportedError","Unknown robot mode. Supported modes are MANUAL ('%s') and AUTO ('%s'). Received a ('%s'). DB query aborted." % (SEGMENT_TYPE_ITO, SEGMENT_TYPE_AUTO, segment_type_id))
            raise ApiError(msg, status_code=400)
        pass
    
    @staticmethod
    def get_segments(leg_id):
        # Return the list of segment data for the specified leg.
        # This SQL query is very specific and thus not derived from a generalization.
        sql_stmt = """
            SELECT segment.id, leg_id, parent_id, starttime_secs, endtime_secs, ito_reason_id, ito_reason.short_description, segment_type_id, segment_type.short_description, obstacle, lighting, slope, ST_X(start_position), ST_Y(start_position), ST_X(local_start_position), ST_Y(local_start_position), orig_starttime_secs
            FROM segment
            FULL JOIN segment_type ON segment.segment_type_id = segment_type.id
            FULL JOIN ito_reason ON segment.ito_reason_id = ito_reason.id
            WHERE leg_id = %s
            ORDER BY segment.id DESC
            """
        return DBInterface.execute(sql_stmt,(leg_id,), True)
    
    @staticmethod
    def store_segment(leg_id, parent_id, ito_reason_id, segment_type_id, starttime_secs, orig_starttime_secs, start_lng = None, start_lat = None, local_x = None, local_y = None):
        # Method to create a segment table data entry either with or without position data.
        param_dict = {
            "leg_id": leg_id, 
            "parent_id": parent_id, 
            "ito_reason_id": ito_reason_id, 
            "segment_type_id": segment_type_id, 
            "starttime_secs": starttime_secs,
            "orig_starttime_secs": orig_starttime_secs
            }
        if start_lat and start_lng and local_x and local_y:
            ApiCommon.insert_table_data_with_position("segment", param_dict, {"start_position": {"pos_x": start_lng, "pos_y": start_lat, "srid": 4326}, "local_start_position": {"pos_x": local_x, "pos_y": local_y, "srid": 0}})
        elif start_lat and start_lng:
            ApiCommon.insert_table_data_with_position("segment", param_dict, {"start_position": {"pos_x": start_lng, "pos_y": start_lat, "srid": 4326}})
        elif local_x and local_y:
            ApiCommon.insert_table_data_with_position("segment", param_dict, {"local_start_position": {"pos_x": local_x, "pos_y": local_y, "srid": 0}})
        else:
            ApiCommon.insert_table_data("segment", param_dict)
        msg = info_msg("Created %s %s segment entry." % ("ITO" if segment_type_id == SEGMENT_TYPE_ITO else "AUTO", "child" if parent_id else "parent"))
        return print_and_return(msg)

    def get(self):
        # This method gets all the data from the segments of a specified leg_id
        # Find leg_id in url or json data.
        param_name = "leg_id"
        leg_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(leg_id)
        # Return the list of segment data for the specified leg.
        return Segment.get_segments(leg_id)

    def post(self):
        # This method creates new entries in the segment table.
        # Get parameter values.
        param_names = ["leg_id","segment_type_id", "ito_reason_id", "lng", "lat", "local_x", "local_y", "orig_starttime_secs"]
        data = ApiCommon.get_all_params_from_json(param_names)
        leg_id = data[param_names[0]]
        segment_type_id = data[param_names[1]]
        ito_reason_id = data[param_names[2]]
        start_lng = data[param_names[3]]
        start_lat = data[param_names[4]]
        local_x = data[param_names[5]]
        local_y = data[param_names[6]]
        orig_starttime_secs = data[param_names[7]]
        starttime_secs = int(time.time())
        # Allow selection of leg_id by string.
        if leg_id == "Current Leg":
            leg_id = ApiCommon.get_open_log_ids("leg", 1)
        else:
            # Check that the leg_id is valid.
            ApiCommon.check_id(leg_id)
            # Check that the requested leg is open.
            ApiCommon.check_open_log("leg", leg_id)
        # Check that the segment_type_id is valid.
        Segment.check_segment_type(segment_type_id)
        # Get current open master segment id and type
        try:
            master_segment_id, master_segment_type_id = Segment.get_current_master_segment(leg_id)
        except ApiError:
            # No currenly open master segment.
            master_segment_type_id = None
        # New segment type is different from current or there is no current master:
        if segment_type_id != master_segment_type_id:
            # Close open segments.
            Segment.close()
            # Create a new master segment.
            msg = Segment.store_segment(leg_id, None, None, segment_type_id, starttime_secs, orig_starttime_secs, start_lng, start_lat, local_x, local_y)
            # Get the new master segment id.
            new_master_segment_id = Segment.get_current_master_segment(leg_id)[0]
            if (segment_type_id == SEGMENT_TYPE_ITO):
                # Immediately create a child segment for any ITO master segment.
                Segment.store_segment(leg_id, new_master_segment_id, ito_reason_id, segment_type_id, starttime_secs, orig_starttime_secs, start_lng, start_lat, local_x, local_y)
            return msg
        # Trying to create an ITO segment with another ITO open will result in the creation of a new child.
        elif master_segment_type_id == SEGMENT_TYPE_ITO:
            return Segment.store_segment(leg_id, master_segment_id, ito_reason_id, segment_type_id, starttime_secs, orig_starttime_secs, start_lng, start_lat, local_x, local_y)
        # Trying to create an AUTO segment with another AUTO open will produce an error.
        else:
            msg = error_msg("UnsupportedError","Unable to create an AUTO segment with another already open. DB query aborted.")
            raise ApiError(msg, status_code=400)
        pass

    def put(self):
        # This method updates an entry in the segment table.
        # Get id and actionfrom JSON data.
        param_names = ["id", "action"]
        params = ApiCommon.get_all_params_from_json(param_names)
        segment_id = params[param_names[0]]
        action = params[param_names[1]]
        # Allow selection of segment_id by string.
        if segment_id == "Current Segment":
            segment_id = Segment.get_current_master_segment()[0]
        else:
            # Check that the id is a number.
            ApiCommon.check_id(segment_id)
        # Check if the request is to edit the data or close the segment entry.
        if action == "edit":
            # Check for any properties to edit
            param_names = ["ito_reason_id", "obstacle", "lighting", "slope"]
            data = ApiCommon.get_any_params_from_json(param_names)
            # Only update if segment is ITO.
            return print_and_return(ApiCommon.update_table_data("segment", data, {"id": segment_id, "segment_type_id": SEGMENT_TYPE_ITO}))
        if action == "close":
            # Close table entry.
            return Segment.close(segment_id)
        pass


class Pose(Resource):

    @staticmethod
    def get_segment_poses(segment_id):
        # Return the GeoJSON data for a specified segment.
        # This SQL query is very specific and thus not derived from a generalization.
        sql_stmt = " \
            SELECT json_build_object( \
                'type', 'FeatureCollection', \
                'features', json_agg( \
                    json_build_object( \
                        'type', 'Feature', \
                        'geometry', ST_AsGeoJSON(pose.position)::json, \
                        'properties', json_build_object( \
                            'id', pose.id, \
                            'segmentId', pose.segment_id, \
                            'type', segment_type.short_description \
                        ) \
                    ) \
                ) \
            ) \
            FROM pose \
            FULL JOIN segment ON pose.segment_id = segment.id \
            FULL JOIN segment_type ON segment.segment_type_id = segment_type.id \
            WHERE segment_id = %s"
        return DBInterface.execute(sql_stmt,(segment_id,), True, True)

    def get(self):
        # This method gets all the position data of a specified segment in GeoJSON format.
        # Find segment_id in url or json data.
        param_name = "segment_id"
        segment_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(segment_id)
        # Check that the segment_id refers to a master segment.
        Segment.check_master_segment(segment_id)
        # Return the GeoJSON data for the specified segment.
        return Pose.get_segment_poses(segment_id)

    def post(self):
        # This method creates a new entry in the pose table.
        # Get parameter values.
        param_names = ["segment_id", "lat", "lng", "pose_source_id", "orig_secs"]
        data = ApiCommon.get_all_params_from_json(param_names)
        data["secs"] = int(time.time())
        # Allow selection of segment_id by string.
        if data[param_names[0]] == "Current Segment":
                data[param_names[0]] = Segment.get_current_master_segment()[0]
        # Check that the id is a number.
        ApiCommon.check_id(data[param_names[0]])
        lat = data.pop("lat")
        lng = data.pop("lng")
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data_with_position("pose", data, {"position": {"pos_x": lng, "pos_y": lat, "srid": 4326}}))


class Image(Resource):

    @staticmethod
    def get_segment_images(segment_id):
        # Return the list of image data for the specified segment.
        # This SQL query is very specific and thus not derived from a generalization.
        sql_stmt = """
            SELECT id, segment_id, secs, image_filename, encode(image_data,'base64'), description, orig_secs
            FROM image
            WHERE segment_id = %s
            ORDER BY id
            """
        return DBInterface.execute(sql_stmt,(segment_id,), True)

    def get(self):
        # This method gets all the images of a specified segment.
        # Find segment_id in url or json data.
        param_name = "segment_id"
        segment_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(segment_id)
        # Check that the segment_id refers to a master segment.
        Segment.check_master_segment(segment_id)
        # Return the images of the specified segment.
        return Image.get_segment_images(segment_id)

    def post(self):
        # This method creates a new entry in the image table.
        # Get parameter values.
        param_names = ["segment_id", "image_filename", "image_data", "description", "orig_secs"]
        data = ApiCommon.get_all_params_from_json(param_names)
        data["secs"] = int(time.time())
        # Allow selection of segment_id by string.
        if data[param_names[0]] == "Current Segment":
                data[param_names[0]] = Segment.get_current_master_segment()[0]
        # Check that the id is a number.
        ApiCommon.check_id(data[param_names[0]])
        # Check that the segment_id refers to a master segment.
        Segment.check_master_segment(data[param_names[0]])
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("image", data))

    def delete(self):
        # This method deletes an entry in the image table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("image")

class Note(Resource):
    def get(self):
        # This method gets all the notes of a specified segment.
        # Find segment_id in url or json data.
        param_name = "segment_id"
        segment_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(segment_id)
        # Return the notes of the specified segment.
        col_names = ["id", "segment_id", "secs", "personnel_id", "note"]
        return ApiCommon.get_table_data("note", col_names, {"segment_id": segment_id}, True)

    def post(self):
        # This method creates a new entry in the note table.
        # Get parameter values.
        param_names = ["segment_id", "personnel_id", "note"]
        data = ApiCommon.get_all_params_from_json(param_names)
        data["secs"] = int(time.time())
        # Check that the id is a number.
        ApiCommon.check_id(data[param_names[0]])
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("note", data))

    def put(self):
        # This method updates an entry in the note table.
        # Get note_id from JSON data.
        param_name = "id"
        note_id = ApiCommon.get_all_params_from_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(note_id)
        # Check for any properties to edit
        param_names = ["note", "personnel_id"]
        data = ApiCommon.get_any_params_from_json(param_names)
        data["secs"] = int(time.time())
        # Update table entry.
        return print_and_return(ApiCommon.update_table_data("note", data, {"id": note_id}))

    def delete(self):
        # This method deletes an entry in the note table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("note")


class RobotState(Resource):
    def get(self):
        # This method gets the current robot state from the segment type.
        # Get current segment type id.
        current_master_segment_type = Segment.get_current_master_segment()[1]
        # Return the short description of the current segment type.
        return ApiCommon.get_table_data("segment_type", ["short_description"], {"id": current_master_segment_type}, None, 1)


class Personnel(Resource):
    def get(self):
        # This method gets all the data from the personnel table.
        return ApiCommon.get_table_data("personnel", ["id", "name", "institution"], None, True)

    def post(self):
        # This method creates a new entry in the personnel table.
        # Get parameter values.
        param_names = ["name", "institution"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("personnel", data))

    def put(self):
        # This method updates an entry in the personnel table.
        # Get personnel_id from JSON data.
        param_name = "id"
        personnel_id = ApiCommon.get_all_params_from_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(personnel_id)
        # Check for any properties to edit
        param_names = ["name", "institution"]
        data = ApiCommon.get_any_params_from_json(param_names)
        # Update table entry.
        return print_and_return(ApiCommon.update_table_data("personnel", data, {"id": personnel_id}))

    def delete(self):
        # This method deletes an entry in the personnel table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("personnel")


class Weather(Resource):

    def get(self):
        # This method gets the relevant data from the weather table.
        return ApiCommon.get_table_data("weather", ["id", "short_description"], None, True)


class PoseSource(Resource):

    def get(self):
        # This method gets all the data from the pose_source table.
        return ApiCommon.get_table_data("pose_source", ["id", "key", "short_description", "long_description"], None, True)

    def post(self):
        # This method creates a new entry in the pose_source table.
        # Get parameter values.
        param_names = ["key", "short_description", "long_description"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("pose_source", data))

    def put(self):
        # This method updates an entry in the pose_source table.
        # Get pose_source_id from JSON data.
        param_name = "id"
        pose_source_id = ApiCommon.get_all_params_from_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(pose_source_id)
        # Check for any properties to edit
        param_names = ["key", "short_description", "long_description"]
        data = ApiCommon.get_any_params_from_json(param_names)
        # Update table entry.
        return print_and_return(ApiCommon.update_table_data("pose_source", data, {"id": pose_source_id}))

    def delete(self):
        # This method deletes an entry in the pose_source table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("pose_source")


class Vehicle(Resource):

    def get(self):
        # This method gets all the data from the vehicle table.
        col_names = ["id", "key", "short_description", "long_description", "institution", "configuration"]
        return ApiCommon.get_table_data("vehicle", col_names, None, True)

    def post(self):
        # This method creates a new entry in the vehicle table.
        # Get parameter values.
        param_names = ["key", "short_description", "long_description", "institution", "configuration"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("vehicle", data))

    def put(self):
        # This method updates an entry in the vehicle table.
        # Get vehicle_id from JSON data.
        param_name = "id"
        vehicle_id = ApiCommon.get_all_params_from_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(vehicle_id)
        # Check for any properties to edit
        param_names = ["key", "short_description", "long_description", "institution", "configuration"]
        data = ApiCommon.get_any_params_from_json(param_names)
        # Update table entry.
        return print_and_return(ApiCommon.update_table_data("vehicle", data, {"id": vehicle_id}))

    def delete(self):
        # This method deletes an entry in the vehicle table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("vehicle")


class Performer(Resource):

    def get(self):
        # This method gets all the data from the performer table.
        return ApiCommon.get_table_data("performer", ["id", "institution"], None, True)

    def post(self):
        # This method creates a new entry in the performer table.
        # Get parameter values.
        param_names = ["institution"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data("performer", data))

    def put(self):
        # This method updates an entry in the performer table.
        # Get performer_id from JSON data.
        param_name = "id"
        performer_id = ApiCommon.get_all_params_from_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(performer_id)
        # Check for any properties to edit
        param_names = ["institution"]
        data = ApiCommon.get_any_params_from_json(param_names)
        # Update table entry.
        return print_and_return(ApiCommon.update_table_data("performer", data, {"id": performer_id}))

    def delete(self):
        # This method deletes an entry in the performer table.
        # The entry id is retrieved from the json data using the table name.
        return ApiCommon.delete_by_id("performer")


class ItoReason(Resource):
    def get(self):
        # This method gets the relevant data from the ito_reason table.
        return ApiCommon.get_table_data("ito_reason", ["id", "short_description"], None, True)


class SegmentType(Resource):
    def get(self):
        # This method gets the relevant data from the segment_type table.
        return ApiCommon.get_table_data("segment_type", ["id", "short_description"], None, True)

class MapImage(Resource):

    @staticmethod
    def get_map_image(shift_id):
        # Return the map image data for the specified shift.
        # This SQL query is very specific and thus not derived from a generalization.
        sql_stmt = """
            SELECT id, shift_id, secs, frame_id, width, height, resolution, ST_X(origin), ST_Y(origin), encode(image_data,'base64'), orig_secs
            FROM map_image
            WHERE shift_id = %s
            """
        return DBInterface.execute(sql_stmt,(shift_id,), True, True)

    def get(self):
        # This method gets the map image of a specified shift.
        # Find shift_id in url or json data.
        param_name = "shift_id"
        shift_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(shift_id)
        # Return the map image of the specified shift.
        return MapImage.get_map_image(shift_id)

    def post(self):
        # This method creates a new entry in the map table.
        # Get parameter values.
        param_names = ["shift_id", "frame_id", "width", "height", "resolution", "origin_x", "origin_y", "image_data", "orig_secs"]
        data = ApiCommon.get_all_params_from_json(param_names)
        data["secs"] = int(time.time())
        # Allow selection of shift_id by string.
        if data[param_names[0]] == "Current Shift":
            data[param_names[0]] = ApiCommon.get_open_log_ids("shift", 1)
        # Check that the id is a number.
        ApiCommon.check_id(data[param_names[0]])
        # Get position data.
        origin_x = data.pop("origin_x")
        origin_y = data.pop("origin_y")
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data_with_position("map_image", data, {"origin": {"pos_x": origin_x, "pos_y": origin_y, "srid": 0}}))

    def put(self):
        # This method updates an entry in the map table.
        # Get shift_id from JSON data.
        param_names = ["shift_id", "orig_secs"]
        data = ApiCommon.get_all_params_from_json(param_names)
        shift_id = data[param_names[0]]
        orig_secs = data[param_names[1]]
        # Allow selection of shift_id by string.
        if shift_id == "Current Shift":
            shift_id = ApiCommon.get_open_log_ids("shift", 1)
        # Check that the id is a number.
        ApiCommon.check_id(shift_id)
        # Check for update of origin position data.
        origin_x = None
        origin_y = None
        try:
            param_names = ["origin_x", "origin_y"]
            data = ApiCommon.get_all_params_from_json(param_names)
            # Get position data.
            origin_x = data.pop("origin_x")
            origin_y = data.pop("origin_y")
        except ApiError:
            pass
        # Check for any properties to edit.
        param_names = ["frame_id", "width", "height", "resolution", "image_data"]
        data = ApiCommon.get_any_params_from_json(param_names)
        # Add time data.
        data["secs"] = int(time.time())
        data["orig_secs"] = orig_secs
        # Update table entry with or without origin position.
        if origin_x and origin_y:
            return print_and_return(ApiCommon.update_table_data_with_position("map_image", data, {"shift_id": shift_id}, "origin", origin_x, origin_y))
        else:
            return print_and_return(ApiCommon.update_table_data("map_image", data, {"shift_id": shift_id}))
        pass

class LocalPose(Resource):

    @staticmethod
    def get_segment_poses(segment_id):
        # Return the local pose data for a specified segment.
        # This SQL query is very specific and thus not derived from a generalization.
        sql_stmt = """
            SELECT local_pose.id, segment_id, secs, frame_id, ST_X(position), ST_Y(position), segment_type.short_description, orig_secs
            FROM local_pose
            FULL JOIN segment ON local_pose.segment_id = segment.id
            FULL JOIN segment_type ON segment.segment_type_id = segment_type.id
            WHERE segment_id = %s
            ORDER BY id
            """
        return DBInterface.execute(sql_stmt,(segment_id,), True)

    def get(self):
        # This method gets all the local position data of a specified segment.
        # Find segment_id in url or json data.
        param_name = "segment_id"
        segment_id = ApiCommon.get_all_params_from_url_or_json([param_name])[param_name]
        # Check that the id is a number.
        ApiCommon.check_id(segment_id)
        # Check that the segment_id refers to a master segment.
        Segment.check_master_segment(segment_id)
        # Return the local position data for the specified segment.
        return LocalPose.get_segment_poses(segment_id)

    def post(self):
        # This method creates a new entry in the local pose table.
        # Get parameter values.
        param_names = ["segment_id", "frame_id", "x", "y", "orig_secs"]
        data = ApiCommon.get_all_params_from_json(param_names)
        data["secs"] = int(time.time())
        # Allow selection of segment_id by string.
        if data[param_names[0]] == "Current Segment":
                data[param_names[0]] = Segment.get_current_master_segment()[0]
        # Check that the id is a number.
        ApiCommon.check_id(data[param_names[0]])
        # Get position data.
        pos_x = data.pop("x")
        pos_y = data.pop("y")
        # Create table entry.
        return print_and_return(ApiCommon.insert_table_data_with_position("local_pose", data, {"position": {"pos_x": pos_x, "pos_y": pos_y, "srid": 0}}))

class GenerateReport(Resource):

    @staticmethod
    def reformat_multi_string(multi_string):
        return multi_string.replace("\n", "\\\\")

    @staticmethod
    def edit_xml(data):
        # Open and parse XML.
        server_path = os.path.dirname(os.path.realpath(__file__))
        config_file_path = server_path+"/../../ftt_report_generator/config/auto_config.xml"
        with open(config_file_path, 'rb') as xml_file:
            root = etree.parse(xml_file).getroot()
        # Edit XML.
        for child in root:
            # Edit tile server name.
            if child.tag == "resource":
                child.set("tile_server", data["tile_server"])
                child.set("zoom_level", data["zoom_level"])
            # Edit database ip.
            if child.tag == "postgis":
                child.set("host", DBInterface.host)
                child.set("port", DBInterface.port)
            # Edit report info.
            if child.tag == "report":
                child.set("name", data["report_name"])
                child.set("version", data["report_version"]) 
                for k in child:
                    if k.tag == "test_event":
                        k.set("id", data["test_event_id"])
                        k.set("min_dur", data["min_duration"])
                        k.set("local", data["use_local_poses"])
                    if k.tag == "recipient":
                        k.set("name", data["recipient_name"])
                        k.set("address", GenerateReport.reformat_multi_string(data["recipient_address"]))
                    if k.tag == "creator":
                        k.set("name", data["creator_name"])
                        k.set("address", GenerateReport.reformat_multi_string(data["creator_address"]))

        # Save file.
        with open(config_file_path, 'wb') as xml_file:
            xml_file.write(etree.tostring(root, pretty_print=True))
            xml_file.close()   

        return

    @staticmethod
    def call_report_generator():
        # Get script path.
        server_path = os.path.dirname(os.path.realpath(__file__))
        report_generator_path = server_path+"/../../ftt_report_generator/src/db2rep.py"
        # Execute script.
        return os.system("bash -c 'source ~/.bashrc && python3 %s ../config/auto_config.xml'" % report_generator_path)

    @staticmethod
    def check_gps_poses(test_event_id):
        sql_stmt = """
            SELECT shift.id as id
            FROM pose
            INNER JOIN segment ON pose.segment_id = segment.id
            INNER JOIN leg ON segment.leg_id = leg.id
            INNER JOIN shift on leg.shift_id = shift.id
            WHERE test_event_id = %s
            GROUP BY shift.id
            ORDER BY shift.id
            """
        gps_shifts = DBInterface.execute(sql_stmt,(test_event_id,), True)
        if not gps_shifts:
            msg = error_msg("UnsupportedError","Test event %s has no GPS data to generate a report." % test_event_id)
            raise ApiError(msg, status_code=400)

    @staticmethod
    def check_local_poses(test_event_id):
        sql_stmt = """
            SELECT shift.id as id
            FROM local_pose
            INNER JOIN segment ON local_pose.segment_id = segment.id
            INNER JOIN leg ON segment.leg_id = leg.id
            INNER JOIN shift on leg.shift_id = shift.id
            WHERE test_event_id = %s
            GROUP BY shift.id
            ORDER BY shift.id
            """
        local_shifts = DBInterface.execute(sql_stmt,(test_event_id,), True)
        if not local_shifts:
            msg = error_msg("UnsupportedError","Test event %s has no local data to generate a report." % test_event_id)
            raise ApiError(msg, status_code=400)

    def post(self):
        # This method calls a report generating script with the user requested parameter values.
        # Get parameter values.
        param_names = ["tile_server", "zoom_level", "report_name", "report_version", "test_event_id", "min_duration", "use_local_poses", "recipient_name", "recipient_address", "creator_name", "creator_address"]
        data = ApiCommon.get_all_params_from_json(param_names)
        # Check that the test event id is a number.
        ApiCommon.check_id(data[param_names[4]])
        # Check that the test event has poses of the specified type (local or gps).
        if data[param_names[6]] == "true":
            GenerateReport.check_local_poses(data[param_names[4]])
        else:
            GenerateReport.check_gps_poses(data[param_names[4]])
        # Write input xml file.
        GenerateReport.edit_xml(data)
        # Call report generator.
        return_code = GenerateReport.call_report_generator()
        # Check report generator return code.
        if return_code:
            msg = error_msg("ExternalError","Report generating script execution failed. Error Code: %s." % (return_code >> 8))
            raise ApiError(msg, status_code=400)
        return print_and_return(info_msg("Created %s report." % data["report_name"]))

class DownloadData(Resource):

    @staticmethod
    def dump_db(output_file):
        # Use pg_dump to export the database contents to an SQL file
        result = subprocess.run([
            "pg_dump",
            f"--dbname=postgresql://postgres:postgres@{DBInterface.host}:{DBInterface.port}/ftt",
            "--create",
            "-f", output_file
        ], check=True)
        return result.returncode

    def get(self):
        # This method calls a database dump and returns the sql file.
        # Define the output file path
        output_file = os.path.abspath("database_dump.sql")
        # Call database dump.
        return_code = DownloadData.dump_db(output_file)
        if return_code:
            msg = error_msg("ExternalError","Database dump failed. Error Code: %s." % return_code)
            raise ApiError(msg, status_code=400)
        try:
            # Serve the file for download
            return send_file(output_file, as_attachment=True)
        except Exception as e:
            msg = error_msg("ExternalError","File send failed. Error: %s." % str(e))
            raise ApiError(msg, status_code=400)
        finally:
            # Clean up the file if needed
            if os.path.exists(output_file):
                os.remove(output_file)

api.add_resource(ItoReason, '/ito_reason', endpoint='ito_reason')

api.add_resource(Personnel, '/personnel', endpoint='personnel')

api.add_resource(Note, '/note', endpoint='note')

api.add_resource(Performer, '/performer', endpoint='performer')

api.add_resource(Vehicle, '/vehicle', endpoint='vehicle')

api.add_resource(Weather, '/weather', endpoint='weather')

api.add_resource(PoseSource, '/pose_source', endpoint='pose_source')

api.add_resource(TestEvent, '/test_event', endpoint='test_event')

api.add_resource(Shift, '/shift', endpoint='shift')

api.add_resource(Leg, '/leg', endpoint='leg')

api.add_resource(Segment, '/segment', endpoint='segment')

api.add_resource(RobotState, '/robot_state', endpoint='robot_state')

api.add_resource(Image, '/image', endpoint='image')

api.add_resource(SegmentType, '/segment_type', endpoint='segment_type')

api.add_resource(Pose, '/pose', endpoint='pose')

api.add_resource(MapImage, '/map_image', endpoint='map_image')

api.add_resource(LocalPose, '/local_pose', endpoint='local_pose')

api.add_resource(GenerateReport, '/generate_report', endpoint='generate_report')

api.add_resource(DownloadData, '/download_data', endpoint='download_data')

@app.errorhandler(ApiError)
def handle_api_error(error):
    response = jsonify(error.to_dict())
    response.status_code = error.status_code
    return response

@app.route("/")
def root():
    return app.send_static_file('index.html')

@app.route("/config")
def config():
    return app.send_static_file('config.html')

@app.route('/report')
def report():
    return send_from_directory('../../ftt_report_generator/build', 'report.pdf')

if __name__ == '__main__':
    # Optional arguments.
    parser=argparse.ArgumentParser()
    parser.add_argument('--server_ip', help='IP on which this server will run')
    parser.add_argument('--server_port', help='Port on which this server will run')
    parser.add_argument('--database_ip', help='IP on which the postgres database runs')
    parser.add_argument('--database_port', help='Port on which the postgres database runs')
    args=parser.parse_args()

    # Initialize the database interface.
    DBInterface.init(args.database_ip, args.database_port)
    # Run the web server.
    server_ip = args.server_ip or '0.0.0.0'
    server_port = args.server_port or 5000
    app.run(debug=True, host=server_ip, port=server_port)
    # Close the database connection.
    print("Closing database connection.")
    DBInterface.close()
    pass
