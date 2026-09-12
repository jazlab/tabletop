"""File-backed HTTP Basic authentication for the noVNC web and WS endpoints."""

import hmac

from websockify.auth_plugins import BasicHTTPAuth


class CredentialsFileAuth(BasicHTTPAuth):
    """Load one ``username:password`` pair from a root-readable file."""

    def __init__(self, src=None):
        if not src:
            raise ValueError("A noVNC credentials file is required")
        with open(src, encoding="utf-8") as credentials_file:
            credentials = credentials_file.readline().rstrip("\r\n")
        if ":" not in credentials:
            raise ValueError("Expected username:password in credentials file")
        username, password = credentials.split(":", 1)
        if not username or not password:
            raise ValueError("noVNC username and password must both be non-empty")
        self._credentials = credentials
        super().__init__(src=credentials)

    def validate_creds(self, username, password):
        """Compare credentials without an early-exit timing leak."""
        return hmac.compare_digest(f"{username}:{password}", self._credentials)
