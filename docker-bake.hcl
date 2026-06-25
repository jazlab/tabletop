group "default" {
    targets = ["novnc", "ros-base"]
}

target "novnc" {
    context = "./docker/novnc"
    dockerfile = "Dockerfile"
    tags = ["jazlabtabletop/novnc"]
}

target "ros-base" {
    context = "."
    dockerfile = "./docker/ros/Dockerfile"
    tags = ["jazlabtabletop/ros-base"]
}
