FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# install requirements
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3 \
    python3-dev \
    python3-wheel \
    python3-setuptools \
    python3-pyproj \
    python3-jinja2 \
    python3-parse \
    python3-lxml \
    python3-matplotlib \
    python3-numpy \
    python3-tk \
    python3-requests \
    python3-pil \
    python3-psycopg2 \
    python3-flask \
    python3-flask-restful \
    python3-flask-cors \
    texlive \
    texlive-lang-german \
    texlive-latex-extra \
    texlive-fonts-extra \
    texlive-xetex \
    libpq-dev \
    ca-certificates \
    curl \
    gnupg \
  && rm -rf /var/lib/apt/lists/*

# make the directory for the web server code (to be mounted as volume)
RUN mkdir /ftt_server

# make the directory for the web application code (to be mounted as volume)
RUN mkdir /ftt_web_interface

# make the directory for the report generator code (to be mounted as volume)
RUN mkdir /ftt_report_generator

# download and install nodejs
ENV NODE_VERSION 22.14.0
RUN ARCH= && dpkgArch="$(dpkg --print-architecture)" \
  && case "${dpkgArch##*-}" in \
    amd64) ARCH='x64';; \
    ppc64el) ARCH='ppc64le';; \
    s390x) ARCH='s390x';; \
    arm64) ARCH='arm64';; \
    armhf) ARCH='armv7l';; \
    i386) ARCH='x86';; \
    *) echo "unsupported architecture"; exit 1 ;; \
  esac \
  && export GNUPGHOME="$(mktemp -d)" \
  && set -ex \
  && for key in \
    C0D6248439F1D5604AAFFB4021D900FFDB233756 \
    DD792F5973C6DE52C432CBDAC77ABFA00DDBF2B7 \
    CC68F5A3106FF448322E48ED27F5E38D5B0A215F \
    8FCCA13FEF1D0C2E91008E09770F7A9A5AE15600 \
    890C08DB8579162FEE0DF9DB8BEAB4DFCF555EF4 \
    C82FA3AE1CBEDC6BE46B9360C43CEC45C17AB93C \
    108F52B48DB57BB0CC439B2997B01419BD92F80A \
    A363A499291CBBC940DD62E41F10027AF002F8B0 \
  ; do \
      gpg --batch --keyserver hkps://keys.openpgp.org --recv-keys "$key" || \
      gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key" ; \
  done \
  && curl -fsSLO --compressed "https://nodejs.org/dist/v$NODE_VERSION/node-v$NODE_VERSION-linux-$ARCH.tar.xz" \
  && curl -fsSLO --compressed "https://nodejs.org/dist/v$NODE_VERSION/SHASUMS256.txt.asc" \
  && gpg --batch --decrypt --output SHASUMS256.txt SHASUMS256.txt.asc \
  && gpgconf --kill all \
  && rm -rf "$GNUPGHOME" \
  && grep " node-v$NODE_VERSION-linux-$ARCH.tar.xz\$" SHASUMS256.txt | sha256sum -c - \
  && tar -xJf "node-v$NODE_VERSION-linux-$ARCH.tar.xz" -C /usr/local --strip-components=1 --no-same-owner \
  && rm "node-v$NODE_VERSION-linux-$ARCH.tar.xz" SHASUMS256.txt.asc SHASUMS256.txt \
  && ln -s /usr/local/bin/node /usr/local/bin/nodejs \
  # smoke tests
  && node --version \
  && npm --version


# config nodejs proxy
RUN npm config set proxy ${http_proxy}
RUN npm config set https-proxy ${https_proxy}

# compile code and run the web server
CMD ["/bin/bash", "-c", "cd ftt_web_interface; npm install; cd ../ftt_server/scripts; python3 api.py --database_ip=ftt-database;"]
