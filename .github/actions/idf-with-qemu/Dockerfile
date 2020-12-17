FROM espressif/idf:release-v4.2

WORKDIR /

ENV QEMU_REL=esp-develop-20200528
ENV QEMU_SHA256=e91e737751de699397c411cc13a1a5699da846bec8922ea887021ea16a94381d
ENV QEMU_DIST=qemu-${QEMU_REL}.tar.bz2
ENV QEMU_URL=https://github.com/espressif/qemu/releases/download/${QEMU_REL}/${QEMU_DIST}

RUN wget --no-verbose ${QEMU_URL} \
  && echo "${QEMU_SHA256} *${QEMU_DIST}" | sha256sum --check --strict - \
  && tar -xf $QEMU_DIST -C /opt \
  && rm ${QEMU_DIST}

ENV PATH=/opt/qemu/bin:${PATH}

# Install QEMU runtime dependencies
RUN apt-get update && apt-get install -y -q \
    libpixman-1-0 \
    libnuma1 \
    libglib2.0-0 \
  && rm -rf /var/lib/apt/lists/*