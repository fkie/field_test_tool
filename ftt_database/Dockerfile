FROM postgis/postgis:13-3.1

# set env variables for postgres
ENV POSTGRES_PASSWORD postgres
ENV POSTGRES_DB ftt
ENV PGDATA /var/lib/postgresql/data/pgdata

# copy db initialization scripts
COPY ftt_database/postgres/ /docker-entrypoint-initdb.d/
