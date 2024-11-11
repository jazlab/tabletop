docker run --rm -d -v $(dirname "$(readlink -f "$0")"):/app -p 5900:5900 -p 6080:6080 --name ursim universalrobots/ursim_e-series
