group "default" {
    targets = ["novnc", "ros-base"]
}

target "novnc" {
    context = "./docker/novnc"
    dockerfile = "Dockerfile"
    tags = ["jazlabtabletop/novnc", "jazlabtabletop/novnc:latest"]
    platforms = ["linux/amd64", "linux/arm64"]
}

target "ros-base" {
    context = "."
    dockerfile = "./docker/ros/Dockerfile"
    tags = ["jazlabtabletop/ros-base", "jazlabtabletop/ros-base:latest"]
    platforms = ["linux/amd64", "linux/arm64"]
}
