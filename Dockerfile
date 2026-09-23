# Debian bookworm (12). Moved off bullseye 2026-09 because the
# bullseye-security pool started 404-ing point-release .debs during
# multi-arch builds (libpython3.9-*, curl, libcurl4). bookworm is
# current-stable, ships the same python3-serial + curl packages, and
# keeps the image compatible with BlueOS's docker runtime.
FROM python:3.11-slim-bookworm

# Install minimal system dependencies. --no-install-recommends keeps the
# image small; retries + fix-missing make apt tolerant of short-lived
# mirror hiccups without silently dropping requested packages.
RUN apt-get update -o Acquire::Retries=3 \
    && apt-get install -y --no-install-recommends --fix-missing \
        python3-serial curl \
    && rm -rf /var/lib/apt/lists/*

# Create app directory
WORKDIR /app

# Create logs directory inside the container
RUN mkdir -p /app/logs && chmod 777 /app/logs

# Copy app files
COPY app/ .

# Download frontend vendor assets at build time so the UI works offline
RUN mkdir -p static/vendor/js static/vendor/css static/vendor/fonts \
    && curl -fsSL -o static/vendor/js/vue.min.js \
       "https://cdn.jsdelivr.net/npm/vue@2.7.16/dist/vue.min.js" \
    && curl -fsSL -o static/vendor/js/vuetify.min.js \
       "https://cdn.jsdelivr.net/npm/vuetify@2.7.2/dist/vuetify.min.js" \
    && curl -fsSL -o static/vendor/js/axios.min.js \
       "https://cdn.jsdelivr.net/npm/axios@1.7.9/dist/axios.min.js" \
    && curl -fsSL -o static/vendor/css/vuetify.min.css \
       "https://cdn.jsdelivr.net/npm/vuetify@2.7.2/dist/vuetify.min.css" \
    && curl -fsSL -o static/vendor/css/materialdesignicons.min.css \
       "https://cdn.jsdelivr.net/npm/@mdi/font@7.4.47/css/materialdesignicons.min.css" \
    && curl -fsSL -o static/vendor/fonts/materialdesignicons-webfont.woff2 \
       "https://cdn.jsdelivr.net/npm/@mdi/font@7.4.47/fonts/materialdesignicons-webfont.woff2" \
    && curl -fsSL -o static/vendor/fonts/roboto-400.woff2 \
       "https://cdn.jsdelivr.net/fontsource/fonts/roboto@latest/latin-400-normal.woff2" \
    && curl -fsSL -o static/vendor/fonts/roboto-500.woff2 \
       "https://cdn.jsdelivr.net/fontsource/fonts/roboto@latest/latin-500-normal.woff2" \
    && curl -fsSL -o static/vendor/fonts/roboto-700.woff2 \
       "https://cdn.jsdelivr.net/fontsource/fonts/roboto@latest/latin-700-normal.woff2" \
    && printf '@font-face{font-family:"Roboto";font-weight:400;font-style:normal;src:url("../fonts/roboto-400.woff2") format("woff2")}\n\
@font-face{font-family:"Roboto";font-weight:500;font-style:normal;src:url("../fonts/roboto-500.woff2") format("woff2")}\n\
@font-face{font-family:"Roboto";font-weight:700;font-style:normal;src:url("../fonts/roboto-700.woff2") format("woff2")}\n' \
       > static/vendor/css/roboto.css

# Install Python dependencies directly (without upgrading pip)
# Install each package separately to avoid hash verification issues
RUN pip install --no-cache-dir flask==2.0.1 && \
    pip install --no-cache-dir pyserial==3.5 && \
    pip install --no-cache-dir requests==2.28.1 && \
    pip install --no-cache-dir Werkzeug==2.0.3 && \
    pip install --no-cache-dir Jinja2==3.0.3 && \
    pip install --no-cache-dir MarkupSafe==2.0.1 && \
    pip install --no-cache-dir itsdangerous==2.0.1 && \
    pip install --no-cache-dir flask-cors==3.0.10 && \
    pip install --no-cache-dir waitress==2.1.2 && \
    pip install --no-cache-dir websockets

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV FLASK_APP=app.py

# Expose ports (6436 = web UI, 8765 = Cockpit data-lake WebSocket)
EXPOSE 6436
EXPOSE 8765

# BlueOS extension metadata
LABEL org.blueos.type="tool"
LABEL org.blueos.version="1.1.6"
LABEL org.blueos.requirements="core >= 1.1"
LABEL org.blueos.name="Airmar WX"
LABEL org.blueos.description="Interface for Airmar WX-series WeatherStations (300WX, 200WX) over NMEA 0183 — live dashboard, ArduPilot UDP, and Cockpit WebSocket output."
LABEL org.blueos.icon="mdi-weather-windy"
LABEL org.blueos.category="Sensors"
LABEL org.blueos.order="10"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "6436/tcp": {},\
    "8765/tcp": {}\
  },\
  "HostConfig": {\
    "CpuPeriod": 100000,\
    "CpuQuota": 100000,\
    "Binds": [\
      "/usr/blueos/extensions/300WX:/app/logs",\
      "/dev:/dev"\
    ],\
    "NetworkMode": "host",\
    "Privileged": true\
  }\
}'

# The BlueOS Extensions Repository processor reads UNPREFIXED labels
# (authors, company, type, readme, links, requirements). Keeping the
# org.blueos.* duplicates above for backward compatibility with anything
# that may still expect them.
LABEL version="1.1.6"
LABEL type="tool"
LABEL requirements="core >= 1.1"

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[\
    {\
        "name": "Tony White",\
        "email": "tony@bluerobotics.com"\
    }\
]'
LABEL org.blueos.authors='[\
    {\
        "name": "Tony White",\
        "email": "tony@bluerobotics.com"\
    }\
]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='{\
    "about": "Airmar WX-series WeatherStation interface for BlueOS",\
    "name": "Blue Robotics",\
    "email": "support@bluerobotics.com"\
}'
LABEL org.blueos.company='{\
    "about": "Airmar WX-series WeatherStation interface for BlueOS",\
    "name": "Blue Robotics",\
    "email": "support@bluerobotics.com"\
}'

ARG REPO
ARG OWNER
LABEL readme='https://raw.githubusercontent.com/vshie/Airmar-WX/{tag}/README.md'
LABEL links='{\
    "source": "https://github.com/vshie/Airmar-WX",\
    "website": "https://bluerobotics.com"\
}'
LABEL org.blueos.readme='https://raw.githubusercontent.com/vshie/Airmar-WX/{tag}/README.md'
LABEL org.blueos.links='{\
    "source": "https://github.com/vshie/Airmar-WX",\
    "website": "https://bluerobotics.com"\
}'

# Run the application
CMD ["python", "main.py"]
