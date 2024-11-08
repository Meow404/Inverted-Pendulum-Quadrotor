FROM robotlocomotion/drake:noble-1.34.0


RUN apt-get update && apt-get upgrade -y
RUN apt-get install python3.12-venv -y

ENV PATH="/opt/drake/bin:${PATH}" 

EXPOSE 8888
USER  meam-5170 1000

WORKDIR /home/stephen/Workspace/fall-2024/MEAM-5170/project
