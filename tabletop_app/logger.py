"""Logger module for the tabletop app.

Import this module with `from logger import logger` to get a logger instance.
"""

import logging

# Create logger instance
logging.basicConfig(format="[%(levelname)s] - %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
