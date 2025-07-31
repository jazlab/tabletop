# ============================================================================= #type: ignore  # noqa E501
# Copyright © 2025 NaturalPoint, Inc. All Rights Reserved.
#
# THIS SOFTWARE IS GOVERNED BY THE OPTITRACK PLUGINS EULA AVAILABLE AT https://www.optitrack.com/about/legal/eula.html #type: ignore  # noqa E501
# AND/OR FOR DOWNLOAD WITH THE APPLICABLE SOFTWARE FILE(S) (“PLUGINS EULA”). BY DOWNLOADING, INSTALLING, ACTIVATING #type: ignore  # noqa E501
# AND/OR OTHERWISE USING THE SOFTWARE, YOU ARE AGREEING THAT YOU HAVE READ, AND THAT YOU AGREE TO COMPLY WITH AND ARE #type: ignore  # noqa E501
# BOUND BY, THE PLUGINS EULA AND ALL APPLICABLE LAWS AND REGULATIONS. IF YOU DO NOT AGREE TO BE BOUND BY THE PLUGINS #type: ignore  # noqa E501
# EULA, THEN YOU MAY NOT DOWNLOAD, INSTALL, ACTIVATE OR OTHERWISE USE THE SOFTWARE AND YOU MUST PROMPTLY DELETE OR #type: ignore  # noqa E501
# RETURN IT. IF YOU ARE DOWNLOADING, INSTALLING, ACTIVATING AND/OR OTHERWISE USING THE SOFTWARE ON BEHALF OF AN ENTITY, #type: ignore  # noqa E501
# THEN BY DOING SO YOU REPRESENT AND WARRANT THAT YOU HAVE THE APPROPRIATE AUTHORITY TO ACCEPT THE PLUGINS EULA ON #type: ignore  # noqa E501
# BEHALF OF SUCH ENTITY. See license file in root directory for additional governing terms and information. #type: ignore  # noqa E501
# ============================================================================= #type: ignore  # noqa E501


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish
# a connection and receive data via that NatNet connection
# to decode it using the NatNetClientLibrary.

import argparse
import sys
import time

import DataDescriptions
import MoCapData
from NatNetClient import NatNetClient

# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.


def receive_new_frame(data_dict):
    order_list = [
        "frameNumber",
        "markerSetCount",
        "unlabeledMarkersCount",  # type: ignore  # noqa F841
        "rigidBodyCount",
        "skeletonCount",
        "labeledMarkerCount",
        "timecode",
        "timecodeSub",
        "timestamp",
        "isRecording",
        "trackedModelsChanged",
    ]
    out_string = "    "
    for key in data_dict:
        out_string += key + "= "
        if key in data_dict:
            out_string += str(data_dict[key]) + " "
        out_string += "/"
    print(out_string)


def receive_new_frame_with_data(data_dict):
    order_list = [
        "frameNumber",
        "markerSetCount",
        "unlabeledMarkersCount",  # type: ignore  # noqa F841
        "rigidBodyCount",
        "skeletonCount",
        "labeledMarkerCount",
        "timecode",
        "timecodeSub",
        "timestamp",
        "isRecording",
        "trackedModelsChanged",
        "offset",
        "mocap_data",
    ]
    out_string = "    "
    for key in data_dict:
        out_string += key + "= "
        if key in data_dict:
            out_string += str(data_dict[key]) + " "
        out_string += "/"
    print(out_string)


# This is a callback function that gets connected to the NatNet client.
# It is called once per rigid body per frame.
def receive_rigid_body_frame(new_id, position, rotation):
    print("Received frame for rigid body", new_id)
    print(
        "Received frame for rigid body", new_id, " ", position, " ", rotation
    )


def add_lists(totals, totals_tmp):
    totals[0] += totals_tmp[0]
    totals[1] += totals_tmp[1]
    totals[2] += totals_tmp[2]
    return totals


def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s" % natnet_client.local_ip_address)
    print("  Server:          %s" % natnet_client.server_ip_address)
    print("  Command Port:    %d" % natnet_client.command_port)
    print("  Data Port:       %d" % natnet_client.data_port)

    changeBitstreamString = "  Can Change Bitstream Version = "
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s" % natnet_client.multicast_address)
        changeBitstreamString += "false"
    else:
        print("  Using Unicast")
        changeBitstreamString += "true"

    # NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" % (application_name))
    print(
        "    MotiveVersion  %d %d %d %d"
        % (
            server_version[0],
            server_version[1],
            server_version[2],
            server_version[3],
        )
    )  # type: ignore  # noqa F501
    print(
        "    NatNetVersion  %d %d %d %d"
        % (
            nat_net_version_server[0],
            nat_net_version_server[1],
            nat_net_version_server[2],
            nat_net_version_server[3],
        )
    )  # type: ignore  # noqa F501
    print("  NatNet Bitstream Requested")
    print(
        "    NatNetVersion  %d %d %d %d"
        % (
            nat_net_requested_version[0],
            nat_net_requested_version[1],  # type: ignore  # noqa F501
            nat_net_requested_version[2],
            nat_net_requested_version[3],
        )
    )  # type: ignore  # noqa F501

    print(changeBitstreamString)
    # print("command_socket = %s" % (str(natnet_client.command_socket)))
    # print("data_socket    = %s" % (str(natnet_client.data_socket)))
    print("  PythonVersion    %s" % (sys.version))


def print_commands(can_change_bitstream):
    outstring = "Commands:\n"
    outstring += "Return Data from Motive\n"
    outstring += "  s  send data descriptions\n"
    outstring += "  r  resume/start frame playback\n"
    outstring += "  p  pause frame playback\n"
    outstring += "     pause may require several seconds\n"
    outstring += "     depending on the frame data size\n"
    outstring += "Change Working Range\n"
    outstring += "  o  reset Working Range to: start/current/end frame 0/0/end of take\n"  # type: ignore  # noqa F501
    outstring += (
        "  w  set Working Range to: start/current/end frame 1/100/1500\n"  # type: ignore  # noqa F501
    )
    outstring += "Return Data Display Modes\n"
    outstring += (
        "  j  print_level = 0 supress data description and mocap frame data\n"  # type: ignore  # noqa F501
    )
    outstring += (
        "  k  print_level = 1 show data description and mocap frame data\n"  # type: ignore  # noqa F501
    )
    outstring += "  l  print_level = 20 show data description and every 20th mocap frame data\n"  # type: ignore  # noqa F501
    outstring += "Change NatNet data stream version (Unicast only)\n"
    outstring += "  3  Request NatNet 3.1 data stream (Unicast only)\n"
    outstring += "  4  Request NatNet 4.1 data stream (Unicast only)\n"
    outstring += "General\n"
    outstring += (
        "  t  data structures self test (no motive/server interaction)\n"  # type: ignore  # noqa F501
    )
    outstring += "  c  print configuration\n"
    outstring += "  h  print commands\n"
    outstring += "  q  quit\n"
    outstring += "\n"
    outstring += "NOTE: Motive frame playback will respond differently in\n"
    outstring += "       Endpoint, Loop, and Bounce playback modes.\n"
    outstring += "\n"
    outstring += (
        "EXAMPLE: PacketClient [serverIP [ clientIP [ Multicast/Unicast]]]\n"  # type: ignore  # noqa F501
    )
    outstring += (
        '         PacketClient "192.168.10.14" "192.168.10.14" Multicast\n'  # type: ignore  # noqa F501
    )
    outstring += '         PacketClient "127.0.0.1" "127.0.0.1" u\n'
    outstring += "\n"
    print(outstring)


def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(
        s_client.command_socket,
        s_client.NAT_REQUEST_MODELDEF,
        "",
        (s_client.server_ip_address, s_client.command_port),
    )  # type: ignore  # noqa F501


def test_classes():
    totals = [0, 0, 0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals = add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals = add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d" % totals[0])
    print("[FAIL] Count = %3.1d" % totals[1])
    print("[SKIP] Count = %3.1d" % totals[2])


def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len = len(arg_list)
    if arg_list_len > 1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len > 2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len > 3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False
        if arg_list_len > 4:
            args_dict["stream_type"] = arg_list[4]
    return args_dict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s",
        "--server_address",
        help="The address of the server to connect to.",
        default="192.168.13.40",
    )
    parser.add_argument(
        "-c",
        "--client_address",
        help="The address of the client to connect to.",
        default="192.168.12.10",
    )
    parser.add_argument("-m", "--use_multicast", action="store_true")
    parser.add_argument("--multicast_address", default="239.255.42.99")
    parser.add_argument("--stream_type", default="d")
    args = parser.parse_args()

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_server_address(args.server_address)
    streaming_client.set_client_address(args.client_address)
    streaming_client.set_multicast_address(args.multicast_address)
    streaming_client.set_use_multicast(args.use_multicast)

    # Streaming client configuration.
    # Calls RB handler on emulator for data transmission.
    # streaming_client.new_frame_listener = receive_new_frame
    # streaming_client.new_frame_with_data_listener = (
    #     receive_new_frame_with_data  # type ignore # noqa E501
    # )
    # streaming_client.rigid_body_listener = receive_rigid_body_frame

    # print instructions
    print("NatNet Python Client 4.3\n")

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    try:
        streaming_client.run()

        time.sleep(1)
        if not streaming_client.connected():
            raise RuntimeError(
                "Could not connect properly.  Check that Motive streaming is on."
            )

        print_configuration(streaming_client)
        print("\n")

        streaming_client.spin()
    except KeyboardInterrupt:
        pass
    finally:
        streaming_client.shutdown()


if __name__ == "__main__":
    main()
