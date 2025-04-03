from tabletop_server.flic_client.aioflic import (
    BluetoothControllerState,
    ButtonConnectionChannel,
    ButtonScanner,
    ConnectionStatus,
    FlicClient,
    ScanWizard,
    ScanWizardResult,
)


class TabletopButtonScanner(ButtonScanner):
    def on_advertisement_packet(
        self,
        bd_addr,
        name,
        rssi,
        is_private,
        already_verified,
        already_connected_to_this_device,
        already_connected_to_other_device,
    ):
        print(
            "Found a private button. Please hold it down for 7 seconds to make it public."
        )


class TabletopScanWizard(ScanWizard):
    def __init__(self, client):
        super().__init__()
        self._client = client

    def on_found_private_button(self):
        print(
            "Found a private button. Please hold it down for 7 seconds to make it public."
        )

    def on_found_public_button(self, bd_addr, name):
        print(
            "Found public button "
            + bd_addr
            + " ("
            + name
            + "), now connecting..."
        )

    def on_button_connected(self, bd_addr, name):
        print("The button was connected, now verifying...")
        self._client.got_button(bd_addr)

    def on_completed(self, result, bd_addr, name):
        print("Scan wizard completed with result " + str(result) + ".")
        if result == ScanWizardResult.WizardSuccess:
            print("Your button is now ready. The bd addr is " + bd_addr + ".")


class TabletopButtonConnectionChannel(ButtonConnectionChannel):
    def on_create_connection_channel_response(self, error, connection_status):
        print(
            f"Create connection channel response: {error} {connection_status}"
        )

    def on_removed(self, removed_reason):
        print(f"Removed: {removed_reason}")

    def on_connection_status_changed(
        self, connection_status, disconnect_reason
    ):
        print(
            self.bd_addr
            + " "
            + str(connection_status)
            + (
                " " + str(disconnect_reason)
                if connection_status == ConnectionStatus.Disconnected
                else ""
            )
        )

    def on_button_up_or_down(self, channel, click_type, was_queued, time_diff):
        print(f"Button up or down: {click_type}")

    def on_button_click_or_hold(
        self, channel, click_type, was_queued, time_diff
    ):
        print(f"Button click or hold: {click_type}")

    def on_button_single_or_double_click(
        self, channel, click_type, was_queued, time_diff
    ):
        print(f"Button single or double click: {click_type}")

    def on_button_single_or_double_click_or_hold(
        self, channel, click_type, was_queued, time_diff
    ):
        print(
            "Simple or Double or hold {} {} {}".format(
                channel.bd_addr, str(click_type), time_diff
            )
        )


class TabletopFlicClient(FlicClient):
    def on_new_verified_button(self, bd_addr: str):
        print(f"New verified button: {bd_addr}")

    def on_no_space_for_new_connection(
        self, max_concurrently_connected_buttons: int
    ):
        print(
            f"No space for new connection: {max_concurrently_connected_buttons}"
        )

    def on_got_space_for_new_connection(
        self, max_concurrently_connected_buttons: int
    ):
        print(
            f"Got space for new connection: {max_concurrently_connected_buttons}"
        )

    def on_bluetooth_controller_state_change(
        self, state: BluetoothControllerState
    ):
        print(f"Bluetooth controller state changed: {state}")

    def on_button_deleted(self, bd_addr: str, deleted_by_this_client: bool):
        print(f"Button deleted: {bd_addr} {deleted_by_this_client}")

    def got_button(self, bd_addr):
        cc = TabletopButtonConnectionChannel(bd_addr, client=self)
        self.add_connection_channel(cc)

    def got_info(self, items):
        print(items)
        for bd_addr in items["bd_addr_of_verified_buttons"]:
            self.got_button(bd_addr)
        self.scan()

    def scan(self):
        print("Starting the scan")
        mywiz = TabletopScanWizard(self)
        self.add_scan_wizard(mywiz)
