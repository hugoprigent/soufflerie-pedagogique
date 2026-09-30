"""Contrôles HTTP de base pour l'interface locale du banc.

Ces contrôles réduisent les requêtes de navigateur intersites. Ils ne
remplacent pas une authentification pour une exposition réseau.
"""

from urllib.parse import urlsplit


def request_allowed(host, origin, fetch_site, bind_host):
    host = (host or "").lower()
    if not host:
        return False
    try:
        hostname = urlsplit("http://" + host).hostname
    except ValueError:
        return False
    if bind_host in ("127.0.0.1", "localhost", "::1") and hostname not in (
        "127.0.0.1", "localhost", "::1"
    ):
        return False
    if fetch_site == "cross-site":
        return False
    if origin:
        try:
            parsed = urlsplit(origin)
        except ValueError:
            return False
        if parsed.scheme not in ("http", "https") or parsed.netloc.lower() != host:
            return False
    return True
