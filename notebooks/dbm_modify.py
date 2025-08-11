# %%
import os

from tabletop_utils import dbm_sqlite3 as dbm

# %%
path = os.path.join(
    os.environ["TABLETOP_DIR"], "ros", "trajectory_cache", "cache.db"
)
db = dbm.open(path, flag="r")

# %%
type(db["rig_hash"])
# %%
db.close()

# %%
