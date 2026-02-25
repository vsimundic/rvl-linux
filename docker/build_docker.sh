# BUILD DOCKER
docker build --progress=plain -t rvl:python . 2>&1 | tee build.log

