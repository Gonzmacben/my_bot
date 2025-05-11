FROM osrf/ros:jazzy-desktop-full

# Instala nano (ya está incluido en el paso siguiente, así que puede omitirse)
RUN apt-get update \
    && apt-get install -y nano \
    && rm -rf /var/lib/apt/lists/*

# Instala git, pip, nano, colcon extensions y build-essential (con nombres correctos y sintaxis corregida)
RUN apt-get update && \
    apt-get install -y \
    git \
    python3-pip \
    nano \
    python3-colcon-common-extensions \
    build-essential && \
    rm -rf /var/lib/apt/lists/*

# Crea el workspace y copia la carpeta launch
WORKDIR /mi_ws
COPY ./launch ./launch

# Da permisos sudo sin contraseña al usuario con UID 1000 (solo si existe)
RUN EXISTING_USER=$(getent passwd 1000 | cut -d: -f1) && \
    echo "${EXISTING_USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${EXISTING_USER} && \
    chmod 0440 /etc/sudoers.d/${EXISTING_USER}

# Copia el entrypoint y lo deja como ejecutable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
CMD ["bash"]