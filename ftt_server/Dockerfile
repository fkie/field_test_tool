FROM ubuntu:18.04

# install requirements
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python2.7 \
    python-dev \
    python-wheel \
    python-pip \
    python-setuptools \
    python-pyproj \
    python-jinja2 \
    python-parse \
    python-lxml \
    texlive \
    texlive-lang-german \
    texlive-latex-extra \
    texmaker \
    texlive-fonts-extra \
    texlive-xetex \
    libpq-dev \
  && python -m pip install -U \
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
CMD ["/bin/bash", "-c", "cd ftt_server/scripts; python api.py --database_ip=ftt-database;"]
