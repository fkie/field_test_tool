FROM postgis/postgis:16-3.4

# set env variables for postgres
ENV POSTGRES_PASSWORD postgres
ENV POSTGRES_DB ftt
ENV PGDATA /var/lib/postgresql/data/pgdata

# copy db initialization scripts
COPY ftt_database/postgres/ /docker-entrypoint-initdb.d/
