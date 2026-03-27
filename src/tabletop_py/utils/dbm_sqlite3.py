"""SQLite3-based key-value database with dbm-compatible interface.

This module provides a persistent key-value store backed by SQLite3,
implementing the MutableMapping interface for dict-like access. It is
designed as a drop-in replacement for the standard dbm module with
better cross-platform support and reliability.

The database stores keys and values as BLOBs, enabling storage of
arbitrary binary data. WAL journal mode is used for improved
concurrent access performance.

Classes:
    error: Exception class for database errors (subclass of OSError).
    _Database: Main database class implementing MutableMapping.

Functions:
    open: Create or open a database file.

Example:
    >>> with open("mydb.sqlite3", flag="c") as db:
    ...     db["key1"] = b"value1"
    ...     print(db["key1"])
    b'value1'

Note:
    Unlike the standard dbm module, this implementation uses SQLite3
    which provides ACID guarantees and better handling of concurrent
    access.
"""

import os
import sqlite3
from collections.abc import MutableMapping
from contextlib import closing, suppress
from pathlib import Path

#: SQL statement to create the key-value table
BUILD_TABLE = """
  CREATE TABLE IF NOT EXISTS Dict (
    key BLOB UNIQUE NOT NULL,
    value BLOB NOT NULL
  )
"""

#: SQL statement to count entries
GET_SIZE = "SELECT COUNT (key) FROM Dict"

#: SQL statement to look up a value by key
LOOKUP_KEY = "SELECT value FROM Dict WHERE key = CAST(? AS BLOB)"

#: SQL statement to insert or replace a key-value pair
STORE_KV = (
    "REPLACE INTO Dict (key, value) VALUES (CAST(? AS BLOB), CAST(? AS BLOB))"
)

#: SQL statement to delete a key
DELETE_KEY = "DELETE FROM Dict WHERE key = CAST(? AS BLOB)"

#: SQL statement to iterate over all keys
ITER_KEYS = "SELECT key FROM Dict"


class error(OSError):
    """Exception raised for database errors.

    Inherits from OSError for compatibility with the standard dbm module
    error handling conventions.
    """

    pass


_ERR_CLOSED = "DBM object has already been closed"
_ERR_REINIT = "DBM object does not support reinitialization"


def _normalize_uri(path):
    """Normalize a file path to a SQLite3 URI.

    Converts a file path to an absolute URI suitable for SQLite3's
    URI filename handling, removing any double slashes.

    Args:
        path: File path to normalize.

    Returns:
        Normalized URI string.
    """
    path = Path(path)
    uri = path.absolute().as_uri()
    while "//" in uri:
        uri = uri.replace("//", "/")
    return uri


class _Database(MutableMapping):
    """SQLite3-backed key-value database with dict-like interface.

    Implements the MutableMapping interface, allowing dict-like access
    to a persistent SQLite3 database. Keys and values are stored as
    BLOBs, supporting arbitrary binary data.

    The database uses autocommit mode and WAL journaling for improved
    performance and concurrent access handling.

    Attributes:
        _cx: SQLite3 connection object, or None if closed.

    Example:
        >>> db = _Database("test.db", flag="c", mode=0o666)
        >>> db["key"] = b"value"
        >>> db["key"]
        b'value'
        >>> db.close()
    """

    def __init__(self, path, /, *, flag, mode, **connect_kwargs):
        """Initialize the database connection.

        Args:
            path: Path to the database file.
            flag: Access mode flag:
                - 'r': Read-only access to existing database.
                - 'w': Read/write access to existing database.
                - 'c': Create if not exists, read/write access.
                - 'n': Always create new (truncate existing), read/write.
            mode: Unix file permissions for new database files.
            **connect_kwargs: Additional arguments passed to sqlite3.connect().

        Raises:
            error: If the database cannot be opened or flag is invalid.
            error: If attempting to reinitialize an existing instance.
        """
        if hasattr(self, "_cx"):
            raise error(_ERR_REINIT)

        path = os.fsdecode(path)
        match flag:
            case "r":
                flag = "ro"
            case "w":
                flag = "rw"
            case "c":
                flag = "rwc"
                Path(path).touch(mode=mode, exist_ok=True)
            case "n":
                flag = "rwc"
                Path(path).unlink(missing_ok=True)
                Path(path).touch(mode=mode)
            case _:
                raise ValueError(
                    f"Flag must be one of 'r', 'w', 'c', or 'n', not {flag!r}"
                )

        # Use URI format for SQLite3 mode specification
        uri = _normalize_uri(path)
        uri = f"{uri}?mode={flag}"

        try:
            self._cx = sqlite3.connect(
                uri, autocommit=True, uri=True, **connect_kwargs
            )
        except sqlite3.Error as exc:
            raise error(str(exc))

        # Enable WAL mode for better concurrent access (optional optimization)
        with suppress(sqlite3.OperationalError):
            self._cx.execute("PRAGMA journal_mode = wal")

        # Create table if opening in create mode
        if flag == "rwc":
            self._execute(BUILD_TABLE)

    def _execute(self, *args, **kwargs):
        """Execute a SQL statement with error handling.

        Args:
            *args: Arguments passed to cursor.execute().
            **kwargs: Keyword arguments passed to cursor.execute().

        Returns:
            Context manager wrapping the cursor for automatic closing.

        Raises:
            error: If the database is closed or SQL execution fails.
        """
        if not self._cx:
            raise error(_ERR_CLOSED)
        try:
            return closing(self._cx.execute(*args, **kwargs))
        except sqlite3.Error as exc:
            raise error(str(exc))

    def __len__(self):
        """Return the number of key-value pairs in the database."""
        with self._execute(GET_SIZE) as cu:
            row = cu.fetchone()
        return row[0]

    def __getitem__(self, key):
        """Retrieve a value by key.

        Args:
            key: The key to look up.

        Returns:
            The value associated with the key.

        Raises:
            KeyError: If the key does not exist.
        """
        with self._execute(LOOKUP_KEY, (key,)) as cu:
            row = cu.fetchone()
        if not row:
            raise KeyError(key)
        return row[0]

    def __setitem__(self, key, value):
        """Store a key-value pair, replacing any existing value.

        Args:
            key: The key to store.
            value: The value to associate with the key.
        """
        self._execute(STORE_KV, (key, value))

    def __delitem__(self, key):
        """Delete a key-value pair.

        Args:
            key: The key to delete.

        Raises:
            KeyError: If the key does not exist.
        """
        with self._execute(DELETE_KEY, (key,)) as cu:
            if not cu.rowcount:
                raise KeyError(key)

    def __iter__(self):
        """Iterate over all keys in the database.

        Yields:
            Keys stored in the database.

        Raises:
            error: If iteration fails due to database error.
        """
        try:
            with self._execute(ITER_KEYS) as cu:
                for row in cu:
                    yield row[0]
        except sqlite3.Error as exc:
            raise error(str(exc))

    def close(self):
        """Close the database connection.

        Safe to call multiple times. After closing, the database
        cannot be used until reopened.
        """
        if self._cx:
            self._cx.close()
            self._cx = None

    def keys(self):
        """Return a list of all keys in the database.

        Returns:
            List of keys (materialized, not a view).
        """
        return list(super().keys())

    def __enter__(self):
        """Enter context manager, returning self."""
        return self

    def __exit__(self, *args):
        """Exit context manager, closing the database."""
        self.close()


def open(filename, /, flag="r", mode=0o666, **connect_kwargs):
    """Open a SQLite3-based key-value database.

    Creates or opens a database file and returns a dict-like object
    for storing and retrieving key-value pairs.

    Args:
        filename: Path to the database file.
        flag: Access mode flag (default 'r'):
            - 'r': Open existing database for read-only access.
            - 'w': Open existing database for read/write access.
            - 'c': Create database if it doesn't exist; read/write access.
            - 'n': Always create new empty database; read/write access.
        mode: Unix file permissions for new database files (default 0o666).
            Only used when creating a new database.
        **connect_kwargs: Additional keyword arguments passed to
            sqlite3.connect(), such as timeout or isolation_level.

    Returns:
        _Database instance implementing MutableMapping interface.

    Raises:
        error: If the database cannot be opened or created.
        ValueError: If flag is not one of 'r', 'w', 'c', or 'n'.

    Example:
        >>> with open("cache.db", flag="c") as db:
        ...     db[b"user:123"] = b"John Doe"
        ...     print(db[b"user:123"])
        b'John Doe'
    """
    return _Database(filename, flag=flag, mode=mode, **connect_kwargs)
