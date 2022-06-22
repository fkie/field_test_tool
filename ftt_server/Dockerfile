FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# install requirements
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3 \
    python3-dev \
    python3-wheel \
    python3-pip \
    python3-setuptools \
    python3-pyproj \
    python3-jinja2 \
    python3-parse \
    python3-lxml \
    python3-matplotlib \
    python3-numpy \
    python3-tk \
    texlive \
    texlive-lang-german \
    texlive-latex-extra \
    texmaker \
    texlive-fonts-extra \
    texlive-xetex \
    libpq-dev \
  && python3 -m pip install -U \
    requests \
    Pillow \
    Flask \
    flask-restful \
    flask-cors \
    psycopg2 \
  && rm -rf /var/lib/apt/lists/*

# make the directory for the web server code (to be mounted as volume)
RUN mkdir /ftt_server

# make the directory for the web application code (to be mounted as volume)
RUN mkdir /ftt_web_interface

# make the directory for the report generator code (to be mounted as volume)
RUN mkdir /ftt_report_generator

# run the web server
CMD ["/bin/bash", "-c", "cd ftt_server/scripts; python3 api.py --database_ip=ftt-database;"]
