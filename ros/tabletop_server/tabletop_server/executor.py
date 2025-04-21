import asyncio

from rclpy.executors import SingleThreadedExecutor


class AIOExecutor(SingleThreadedExecutor):
    async def spin(self):
        while self._context.ok() and not self._is_shutdown:
            self.spin_once(timeout_sec=0)
            await asyncio.sleep(1e-4)
