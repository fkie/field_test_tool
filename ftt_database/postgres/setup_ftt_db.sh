#!/bin/bash

# Set the password for the postgres user
export PGPASSWORD="postgres"

# Query the host and port from PostgreSQL
HOST=$(sudo -u postgres psql -qtA -c "SHOW listen_addresses;" | tr -d '[:space:]')
PORT=$(sudo -u postgres psql -qtA -c "SHOW port;" | tr -d '[:space:]')

# If HOST is '*' (PostgreSQL listens on all interfaces), default to localhost for connection
if [ "$HOST" == "*" ]; then
  HOST="localhost"
fi

# Check if the ftt database exists
DB_EXISTS=$(sudo -u postgres psql -lqt | cut -d \| -f 1 | grep -w ftt)

if [ -z "$DB_EXISTS" ]; then
  # Create ftt database and set up extensions if it doesn't exist
  sudo -u postgres psql -c "ALTER USER postgres PASSWORD 'postgres';" -c "CREATE DATABASE ftt;" -c "\c ftt" -c "CREATE EXTENSION postgis;" -c "CREATE EXTENSION postgis_topology;"
else
  echo "Database 'ftt' already exists. Skipping creation."
fi

# Check if the 'test_event' table exists in the 'ftt' database
TABLE_EXISTS=$(psql -h "$HOST" -p "$PORT" -d ftt -U postgres -tAc "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'test_event');")

if [ "$TABLE_EXISTS" == "t" ]; then
  echo "Database 'ftt' already configured. Skipping schema creation and table setup."
else
  # Execute the SQL scripts to create the database schema and fill tables
  psql -h "$HOST" -p "$PORT" -d ftt -U postgres -a -q -f "$(dirname "${BASH_SOURCE[0]}")/ftt_schema.sql"
  psql -h "$HOST" -p "$PORT" -d ftt -U postgres -a -q -f "$(dirname "${BASH_SOURCE[0]}")/setup_queries.sql"
fi
