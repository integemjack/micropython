FROM espressif/idf:v4.4

RUN git clone -b idfv4.4 https://github.com/espressif/esp-who.git ./esp-who && \
cd esp-who && \
git submodule update --init --recursive && \
mv components/* /opt/esp/idf/components/

WORKDIR /project


#COPY . /project

#RUN cd ports/esp32 && make BOARD=PYDRONE


ENTRYPOINT ["/opt/esp/entrypoint.sh"]

CMD ["/bin/bash"]