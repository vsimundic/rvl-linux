# RUN DOCKER
docker rm rvl_python
docker run \
    --ipc=host \
    --gpus all \
    --runtime=runc \
    --interactive -it \
    --shm-size=10gb \
    --env="DISPLAY" \
    --volume="/media/valentin/T7/FERIT/DOCKER/RVL_DOCKER_new/rvl-linux:/home/RVLuser/rvl-linux" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --workdir="/home/RVLuser/rvl-linux" \
    -p 6018:6018 \
    --privileged \
    --name=rvl_python rvl:python
# docker exec -it rvl bash
