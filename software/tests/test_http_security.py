import unittest

from src.http_security import request_allowed


class RequestAllowedTests(unittest.TestCase):
    def test_local_browser_request(self):
        self.assertTrue(request_allowed(
            "127.0.0.1:8080", "http://127.0.0.1:8080", "same-origin", "127.0.0.1"))

    def test_cross_site_form_post_rejected(self):
        self.assertFalse(request_allowed(
            "127.0.0.1:8080", "https://example.org", "cross-site", "127.0.0.1"))

    def test_dns_rebinding_host_rejected_on_loopback(self):
        self.assertFalse(request_allowed(
            "malicious.example:8080", None, None, "127.0.0.1"))

    def test_lan_mode_accepts_same_origin(self):
        self.assertTrue(request_allowed(
            "192.0.2.10:8080", "http://192.0.2.10:8080", "same-origin", "0.0.0.0"))

    def test_different_origin_rejected_even_if_same_site(self):
        self.assertFalse(request_allowed(
            "127.0.0.1:8080", "http://127.0.0.1:9000", "same-site", "127.0.0.1"))

    def test_malformed_host_rejected(self):
        self.assertFalse(request_allowed("[invalid", None, None, "127.0.0.1"))


if __name__ == "__main__":
    unittest.main()
