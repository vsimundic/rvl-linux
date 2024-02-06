# RUN DOCKER
docker stop rvl_ur5_detectron2

docker rm rvl_ur5_detectron2

docker run --ipc=host --gpus all --runtime=runc --interactive -it \
--shm-size=10gb \
--env="DISPLAY" \
--volume="${PWD}:/home/RVLuser" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/dev:/dev" \
--workdir="/home/RVLuser" \
--privileged \
--net=host \
--name=rvl_ur5_detectron2 rvl:ur5_detectron2

# docker exec -it rvl_ur5_detectron2 bash