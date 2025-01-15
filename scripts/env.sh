# Get directory of current script
SCRIPT_DIR=$(dirname $(readlink -f $0))
source $SCRIPT_DIR/utils.sh

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot_ip)
            if [[ ! "$2" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                echo "Error: Invalid IP address format for --robot_ip: $2"
                exit 1
            fi
            ROBOT_IP="$2"
            shift 2
            ;;
        --reverse_ip)
            if [[ ! "$2" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                echo "Error: Invalid IP address format for --reverse_ip: $2" 
                exit 1
            fi
            REVERSE_IP="$2"
            shift 2
            ;;
        --simulated)
            SIMULATED=true
            shift
            ;;
        *)
            echo "Error: Unknown argument: $1"
            exit 1
            ;;
    esac
done

SIMULATED=${SIMULATED:-false}

if [ "$SIMULATED" = true ] && [ -n "$ROBOT_IP" -o -n "$REVERSE_IP" ]; then
    echo "Error: --simulated cannot be used with --robot_ip or --reverse_ip"
    exit 1
fi

# Set IP addresses based on simulation mode
if [ "$SIMULATED" = true ]; then
    ROBOT_IP_PREFIX="192.168.12"
else
    ROBOT_IP_PREFIX="192.168.13" 
fi

# Set defaults if not provided
ROBOT_IP=${ROBOT_IP:-"$ROBOT_IP_PREFIX.10"}
REVERSE_IP=${REVERSE_IP:-"$ROBOT_IP_PREFIX.11"}