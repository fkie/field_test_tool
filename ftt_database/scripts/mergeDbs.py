#!/usr/bin/env python3
"""mergeDbs.py: script to merge FTT databases."""

__author__ = "Johannes Pellenz, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2022, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import os
import sys
import re
import subprocess

import psycopg2
import psycopg2.sql
import psycopg2.extras

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


# Catalog tables
catalog_tables = ["ito_reason", "segment_type", "weather"]

# Core tables
config_tables = ["performer", "personnel", "pose_source", "vehicle"]
data_tables = ["test_event", "shift", "leg", "segment", "note", "image", "pose", "local_pose", "map_image"]

# All tables
all_tables = catalog_tables + config_tables + data_tables

# key_colums defines which fields have to be modified. It contains the ID fields, and the corresponding foreign keys.
key_columns = {"test_event": [("test_event", "id"), ("shift", "test_event_id")],
               "shift": [("shift", "id"), ("leg", "shift_id"), ("map_image","shift_id")],
               "leg": [("leg", "id"), ("segment", "leg_id")], 
               "segment": [("segment", "id"), ("segment", "parent_id"), ("note","segment_id"), ("image","segment_id"), ("pose","segment_id"), ("local_pose","segment_id")],
               "note": [("note", "id")], 
               "image": [("image", "id")], 
               "pose": [("pose", "id")], 
               "local_pose": [("local_pose", "id")],
               "map_image": [("map_image", "id")],
               "performer": [("performer", "id"), ("shift", "performer_id")],
               "personnel": [("personnel", "id"), ("shift","test_administrator_id"), ("shift","test_director_id"), ("shift","safety_officer_id"), ("shift","robot_operator_id"), ("note","personnel_id")],
               "pose_source": [("pose_source", "id"), ("leg","default_pose_source_id"), ("pose","pose_source_id")],
               "vehicle": [("vehicle", "id"), ("shift","vehicle_id")]
}

# migration dictionary: matches table/column of input with table/column of output tables
# migration_source_dest = { ("image","image"):("filename"), ("image", "image_file"):("data"), ("note", "user"):("user_id")}
migration_source_dest = {}

################################################################
# Helper functions
################################################################

def restore_database_from_dump(dump_file, temp_db_name="ftt_temp"):
    try:
        # Create the temporary database
        subprocess.run(["createdb", temp_db_name, "-U", "postgres", "-h", "localhost"], check=True)

        # Restore the dump into the temporary database
        subprocess.run([
            "psql", "-U", "postgres", "-h", "localhost", "-d", temp_db_name, "-f", dump_file
        ], check=True)

        return f"host=localhost dbname={temp_db_name} user=postgres password=postgres"

    except Exception as e:
        print(f"Error restoring database from dump: {e}")
        raise

def delete_temp_database(temp_db_name="ftt_temp"):
    try:
        subprocess.run(["dropdb", temp_db_name, "-U", "postgres", "-h", "localhost"], check=True)
    except Exception as e:
        print(f"Error deleting temporary database: {e}")

def get_min_id (cursor, table_name):
    query = psycopg2.sql.SQL(
        "SELECT min (id) \
        FROM {}"
    ).format(psycopg2.sql.Identifier(table_name))
    cursor.execute(query)
    result = cursor.fetchone()
    return result[0]

def get_max_id (cursor, table_name):
    query = psycopg2.sql.SQL(
        "SELECT max (id) \
        FROM {}"
    ).format(psycopg2.sql.Identifier(table_name))
    cursor.execute(query)
    result = cursor.fetchone()
    return result[0]

def get_id_key (cursor, table_name):
    query = psycopg2.sql.SQL(
        "SELECT id, key \
        FROM {} \
        ORDER by id"
    ).format(psycopg2.sql.Identifier(table_name))
    cursor.execute(query)
    rows = []
    for row in cursor.fetchall():
        rows.append(row)
    return rows

def get_entries (cursor, table_name):
    query = psycopg2.sql.SQL(
        "SELECT * \
        FROM {} \
        ORDER by id"
    ).format(psycopg2.sql.Identifier(table_name))
    cursor.execute(query)
    rows = []
    for row in cursor.fetchall():
        rows.append(row)
    return rows


def get_table_info (cursor, table_name):
    query = "SELECT column_name, data_type \
        FROM information_schema.columns \
        WHERE table_name = %s"
    cursor.execute(query, (table_name, ))
    rows = []
    for row in cursor.fetchall():
        name = row[0]
        type = row[1]
        if type.upper().startswith("CHAR") or type.upper().startswith("TIMESTAMP") :
            type = "TEXT"
        rows.append ( (name, type) )
    sorted_rows = sorted(rows, key=lambda r: r[1])
    return sorted_rows

def migrate_source_id_key (table_name, table_info_rows):
    new_table_info_rows = []
    for row in table_info_rows:
        col_name = row[0]
        type = row[1]
        if (table_name, col_name) in migration_source_dest:
            col_name = migration_source_dest [table_name, col_name]
        new_table_info_rows.append ( (col_name, type) )
    sorted_rows = sorted(new_table_info_rows, key=lambda r: r[1])
    return sorted_rows

def tableExists (cursor, table_name):
    query = "SELECT to_regclass(%s)" 
    cursor.execute(query, ("public." + table_name,))
    row = cursor.fetchone()
    result = row[0] != None
    return result

################################################################
# Checks for identical contents of the catalog tables
################################################################

def check_id_key (source_cursor, dest_cursor, table_name):
    consistent = True
    print(" - Checking table", table_name, ": ", end='')
    source_id_key = get_id_key (source_cursor, table_name)
    dest_id_key = get_id_key (dest_cursor, table_name)
    # print("Contents ID in source:", source_id_key, "/ in dest:", dest_id_key)
    diff = list ((set(source_id_key) - set(dest_id_key))) + \
           list ((set(dest_id_key) - set(source_id_key)))
    if diff == []:
        print(bcolors.OKGREEN + "OK"+ bcolors.ENDC)
    else:
        print(bcolors.FAIL + "Diff:", diff, bcolors.ENDC)
        consistent = False
    return consistent

################################################################
# Checks for identical columns in tables to be copies
################################################################

def check_columns (source_cursor, dest_cursor, table_name):
    consistent = True
    print(" - Checking table", table_name, ": ", end='')
    source_table_info = get_table_info (source_cursor, table_name)
    source_table_info = migrate_source_id_key (table_name, source_table_info)
    dest_table_info = get_table_info (dest_cursor, table_name)
    diff = list ((set(source_table_info) - set(dest_table_info))) + \
           list ((set(dest_table_info) - set(source_table_info)))
    if diff == []:
        print(bcolors.OKGREEN + "OK"+ bcolors.ENDC)
    else:
        print(bcolors.FAIL + "NOK"+ bcolors.ENDC)
        only_source = list ((set(source_table_info) - set(dest_table_info)))
        if only_source != []:
            print(bcolors.FAIL, "  In source, not in dest (will cause error): ", only_source, bcolors.ENDC)
        only_dest = list ((set(dest_table_info) - set(source_table_info)))
        if only_dest != []:
            print(bcolors.WARNING, "  In dest, not in source (will stay NULL): ", only_dest, bcolors.ENDC)
        consistent = False
    return consistent

################################################################
# Finds config entries in the source DB already in the target
################################################################

def find_duplicated_config (source_cursor, dest_cursor, table_name):
    dupl_pairs = []
    # Get table data
    source_id_data = get_entries (source_cursor, table_name)
    dest_id_data = get_entries (dest_cursor, table_name)
    # Remove the IDs and format the string elements
    source_data = [tuple((el.replace(" ", "").lower() if isinstance(el, str) else el) for el in row[1:]) for row in source_id_data]
    dest_data = [tuple((el.replace(" ", "").lower() if isinstance(el, str) else el) for el in row[1:]) for row in dest_id_data]
    # Find the duplicated data through set difference
    duplicated = list(set(source_data) - (set(source_data) - set(dest_data)))
    # Extract the ids of the duplicated entries
    for dupl_config in duplicated:
        dupl_pairs.append(
            ( source_id_data[source_data.index(dupl_config)][0], dest_id_data[dest_data.index(dupl_config)][0] )
        )
    return dupl_pairs # [(source_id, dest_id), ...]

################################################################
# Check the databases
################################################################

def sanity_check (source_cursor, dest_cursor):
    consistent = True
    print(">>> Checking if ID and Key of catalog tables are identical")
    for table in catalog_tables:
        consistent = check_id_key (source_cursor, dest_cursor, table) and consistent
    print(">>> Checking for identical columns of tables to be copied")
    for table in all_tables:
        consistent = check_columns (source_cursor, dest_cursor, table) and consistent

    return (consistent)

################################################################
# Core function: Merge the databases
################################################################

def merge_dbs (source_conn, dest_conn):
    source_cursor = source_conn.cursor()
    dest_cursor = dest_conn.cursor()
    print(">>> Merging the databases...")

    # Storage for duplicated config entries between source and dest database
    dupl_id_pairs = {} # {'table_name':[(source_id, dest_id), ...], ...}

    dest_max_id_reg = {} # dict to hold the original max ids from the destination db tables
    safe_max_id_reg = {} # dict to hold the original safe max ids from the source db tables
    source_id_offset_reg = {} # dict to hold the original id offset from the source db tables

    # Modify table constraints to allow "id inconsistencies" during intermediate updates (before a "commit")
    for table_name in data_tables + config_tables:
        if tableExists (source_cursor, table_name):
            table_col = key_columns [table_name]
            # If this key is further referenced, make its foreign key constraint deferrable
            for table_col_pair in table_col[1:]:
                mod_table = table_col_pair [0]
                mod_column = table_col_pair [1]
                alter_stmt = psycopg2.sql.SQL(
                    "ALTER TABLE {} ALTER CONSTRAINT {} DEFERRABLE INITIALLY IMMEDIATE").format(
                        psycopg2.sql.Identifier(mod_table), 
                        psycopg2.sql.Identifier(mod_table + "_" + mod_column + "_fkey")
                    )
                source_cursor.execute(alter_stmt)
    
    source_conn.commit()

    # Modify table ids to avoid collisions with ids in the destination database tables
    set_stmt = "SET CONSTRAINTS ALL DEFERRED"
    source_cursor.execute(set_stmt)

    for table_name in config_tables + data_tables:
        if tableExists (source_cursor, table_name):
            print("    * Adjusting ID of table", table_name, "in all tables of source db")
            dest_max_id = get_max_id (dest_cursor, table_name)
            dest_max_id_reg[table_name] = dest_max_id
            print("      - Max ID from target table:", dest_max_id, end='')
            if (dest_max_id == None):
                dest_id_offest = 0
            else:
                dest_id_offest = dest_max_id + 1

            source_min_id = get_min_id (source_cursor, table_name)
            print(", min ID from source table:", source_min_id, end='')
            if (source_min_id != None):
                dest_id_offest = dest_id_offest - source_min_id
            print("/ dest_id_offest is", dest_id_offest)

            source_max_id = get_max_id (source_cursor, table_name)
            print("      - Max ID from source table:", source_max_id, end='') 
            if (source_max_id == None):
                source_id_offset = 0
            else:
                source_id_offset = source_max_id + 1
            print(" / source_id_offset is", source_id_offset, "(will be removed later)")
            source_id_offset_reg[table_name] = source_id_offset

            safe_max_id = dest_id_offest + source_id_offset # avoid problem with PK in source table
            safe_max_id_reg[table_name] = safe_max_id

            # For config tables, check if the config data is already in the target db
            if table_name in config_tables:
                dupl_id_pairs[table_name] = find_duplicated_config(source_cursor, dest_cursor, table_name)
                if dupl_id_pairs[table_name]:
                    print(bcolors.WARNING + "      - Duplicated config entries found. Source -> Target id mappings are: %s" % dupl_id_pairs[table_name] +  bcolors.ENDC)

            key_col_dict = key_columns [table_name]
            for key_col in key_col_dict:
                mod_table = key_col [0]
                mod_column = key_col [1]
                if tableExists (source_cursor, mod_table):
                    print("      - Changing table", mod_table, "column", mod_column)
                    
                    update_stmt = psycopg2.sql.SQL(
                        "UPDATE {} \
                        SET {} = {} + %s"
                    ).format(
                        psycopg2.sql.Identifier(mod_table),
                        psycopg2.sql.Identifier(mod_column),
                        psycopg2.sql.Identifier(mod_column)
                    )
                    source_cursor.execute(update_stmt, (safe_max_id, ))

                    # If there are duplicated configs, set source id to target id.
                    # Copying of the entry in the main config table will later be ignored.
                    if table_name in config_tables and dupl_id_pairs[table_name]:
                        for id_pair in dupl_id_pairs[table_name]:
                            # Set the duplicated entries to target id
                            update_stmt = psycopg2.sql.SQL(
                                "UPDATE {} \
                                SET {} = %s \
                                WHERE {} = %s"
                            ).format(
                                psycopg2.sql.Identifier(mod_table),
                                psycopg2.sql.Identifier(mod_column),
                                psycopg2.sql.Identifier(mod_column),
                            )
                            source_cursor.execute(update_stmt, (id_pair[1], id_pair[0] + safe_max_id))
                        # Set the rest according to the offset
                        update_stmt = psycopg2.sql.SQL(
                            "UPDATE {} \
                            SET {} = {} - %s \
                            WHERE {} > %s"
                        ).format(
                            psycopg2.sql.Identifier(mod_table),
                            psycopg2.sql.Identifier(mod_column),
                            psycopg2.sql.Identifier(mod_column),
                            psycopg2.sql.Identifier(mod_column)
                        )
                        source_cursor.execute(update_stmt, (source_id_offset, dest_max_id))
                    else:
                        update_stmt = psycopg2.sql.SQL(
                            "UPDATE {} \
                            SET {} = {} - %s"
                        ).format(
                            psycopg2.sql.Identifier(mod_table),
                            psycopg2.sql.Identifier(mod_column),
                            psycopg2.sql.Identifier(mod_column)
                        )
                        source_cursor.execute(update_stmt, (source_id_offset, ))


    # Copy source table data to destination database
    for table_name in config_tables + data_tables:
        if tableExists (source_cursor, table_name):
            print("    * Copying contents of table", table_name, "to target db")
            lower_id_limit = dest_max_id_reg[table_name] or 0 # only select data with id higher than the max in the target table (skips duplicated config)
            query = psycopg2.sql.SQL(
                "SELECT * \
                FROM {} \
                WHERE id > %s \
                ORDER BY id"
            ).format(psycopg2.sql.Identifier(table_name))
            source_cursor.execute(query, (lower_id_limit, ))
            col_names = [desc[0] for desc in source_cursor.description]
            for row in source_cursor.fetchall():
                insert_stmt = psycopg2.sql.SQL(
                    "INSERT INTO {} ({}) \
                    VALUES ({})"
                ).format(
                    psycopg2.sql.Identifier(table_name),
                    psycopg2.sql.SQL(', ').join(map(psycopg2.sql.Identifier, col_names)),
                    psycopg2.sql.SQL(', ').join(psycopg2.sql.Placeholder() * len(col_names))
                )
                dest_cursor.execute (insert_stmt, row) 
            # Set auto increment counter to the new max id of the table
            # (The manual insertion with id broke the sequence)
            set_stmt = "SELECT setval('{0}_id_seq', (SELECT MAX(id) FROM {0}))".format(table_name)
            dest_cursor.execute(set_stmt)
            

    # Restore original source database ids
    set_stmt = "SET CONSTRAINTS ALL DEFERRED"
    source_cursor.execute(set_stmt)

    for table_name in config_tables + data_tables:
        if tableExists (source_cursor, table_name):
            print("    * Restoring ID of table", table_name, "in all tables of source db")

            key_col_dict = key_columns [table_name]
            for key_col in key_col_dict:
                mod_table = key_col [0]
                mod_column = key_col [1]
                if tableExists (source_cursor, mod_table):
                    print("      - Restoring table", mod_table, "column", mod_column)
                    
                    # If there are duplicated configs, set the id to what was stored in the dupl_id_pairs.
                    if table_name in config_tables and dupl_id_pairs[table_name]:
                        # Reset the non-duplicated ids according to the stored offset
                        update_stmt = psycopg2.sql.SQL(
                            "UPDATE {table} \
                            SET {col} = {col} + %s \
                            WHERE {col} > %s"
                        ).format(
                            table=psycopg2.sql.Identifier(mod_table),
                            col=psycopg2.sql.Identifier(mod_column)
                        )
                        source_cursor.execute(update_stmt, (source_id_offset_reg[table_name], dest_max_id_reg[table_name]))
                        # Set the duplicated entries to the stored source id (plus stored safe_max_id, substracted later)
                        for id_pair in dupl_id_pairs[table_name]:
                            update_stmt = psycopg2.sql.SQL(
                                "UPDATE {table} \
                                SET {col} = %s \
                                WHERE {col} = %s"
                            ).format(
                                table=psycopg2.sql.Identifier(mod_table),
                                col=psycopg2.sql.Identifier(mod_column)
                            )
                            source_cursor.execute(update_stmt, (id_pair[0] + safe_max_id_reg[table_name], id_pair[1]))
                    else:
                        update_stmt = psycopg2.sql.SQL(
                            "UPDATE {table} \
                            SET {col} = {col} + %s"
                        ).format(
                            table=psycopg2.sql.Identifier(mod_table),
                            col=psycopg2.sql.Identifier(mod_column)
                        )
                        source_cursor.execute(update_stmt, (source_id_offset_reg[table_name], ))

                    # Substract the initially added safe_max_id to obtain the original id
                    update_stmt = psycopg2.sql.SQL(
                        "UPDATE {table} \
                        SET {col} = {col} - %s"
                    ).format(
                        table=psycopg2.sql.Identifier(mod_table),
                        col=psycopg2.sql.Identifier(mod_column)
                    )
                    source_cursor.execute(update_stmt, (safe_max_id_reg[table_name], ))

                    

    source_conn.commit()
    dest_conn.commit()

    print(bcolors.OKBLUE + "Done." +  bcolors.ENDC)
        
################################################################
# Main
################################################################


def main(args):
    # Get connection info from string arguments in the format "host=%s dbname=%s user=%s password=%s"
    source_db_conn_str = sys.argv[1]
    dest_db_conn_str = sys.argv[2]

    # Determine if the source argument is a dump file
    if os.path.isfile(source_db_conn_str):
        print(">>> Detected dump file. Creating and restoring temporary database...")
        dump_file = source_db_conn_str
        source_db_conn_str = restore_database_from_dump(dump_file)

    # Connect to the databases
    print(">>> Connecting to databases...")
    source_conn = psycopg2.connect(source_db_conn_str)
    source_cursor = source_conn.cursor()
    dest_conn = psycopg2.connect(dest_db_conn_str)
    dest_cursor = dest_conn.cursor()

    # Perform a sanity check to ensure the databases are consistent
    print(">>> Checking consistency...")
    do_merge = True
    if sanity_check (source_cursor, dest_cursor):
        print("Ready to merge.")
    else:
        print(bcolors.FAIL + "The databases are not consistent, so it is dangerous to merge!" + bcolors.ENDC)
        ask_confirm = input(bcolors.FAIL + "Type Y if you want to continue anyway, or N to cancel: "+ bcolors.ENDC)    
        do_merge = (ask_confirm == "Y")
    if do_merge:
        ask_confirm = input("Ready to merge %s into %s. Type Y to continue, or N to cancel: " % (
            re.search(r"(?<=host=).*?(?=\s)", source_db_conn_str).group(0) + ":" + re.search(r"(?<=dbname=).*?(?=\s)", source_db_conn_str).group(0), 
            re.search(r"(?<=host=).*?(?=\s)", dest_db_conn_str).group(0) + ":" + re.search(r"(?<=dbname=).*?(?=\s)", dest_db_conn_str).group(0)
        ))    
        do_merge = (ask_confirm == "Y")
        if do_merge:
            merge_dbs (source_conn, dest_conn)

    # Close connections
    source_cursor.close()
    source_conn.close()
    dest_cursor.close()
    dest_conn.close()

    # If a temporary database was created, delete it
    if os.path.isfile(sys.argv[1]):
        print(">>> Deleting temporary database...")
        delete_temp_database()

if __name__ == "__main__":
    main(sys.argv[1:])
