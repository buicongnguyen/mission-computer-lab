"""User-space signed artifact policy, NOT Qualcomm secure boot or an OTA installer."""

import hashlib
import json
from pathlib import Path
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey
from cryptography.hazmat.primitives.serialization import Encoding, PublicFormat
from cryptography.exceptions import InvalidSignature


def canonical(manifest):
    return json.dumps(manifest, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def sign(payload, key, version=2, device='wsl-mission-sim'):
    manifest = {'version': version, 'device': device, 'sha256': hashlib.sha256(payload).hexdigest()}
    return manifest, key.sign(canonical(manifest))


def verify(payload, manifest, signature, public_key, minimum_version=2, device='wsl-mission-sim'):
    public_key.verify(signature, canonical(manifest))
    if set(manifest) != {'version', 'device', 'sha256'}:
        raise ValueError('manifest_schema')
    if type(manifest['version']) is not int or manifest['version'] < minimum_version:
        raise ValueError('rollback_rejected')
    if manifest['device'] != device:
        raise ValueError('device_mismatch')
    if hashlib.sha256(payload).hexdigest() != manifest['sha256']:
        raise ValueError('payload_tampered')
    return True


def sidecar(path, suffix):
    return path.with_name(path.name + suffix)


def provision(path, key, version=2, device='wsl-mission-sim'):
    """Release step: sign the stored artifact and write a detached manifest and signature."""
    path = Path(path)
    manifest, signature = sign(path.read_bytes(), key, version, device)
    sidecar(path, '.manifest.json').write_bytes(canonical(manifest))
    sidecar(path, '.sig').write_bytes(signature)


def public_key_hex(key):
    return key.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw).hex()


def load_public_key(path):
    return Ed25519PublicKey.from_public_bytes(bytes.fromhex(Path(path).read_text().strip()))


def boot_gate(path, public_key):
    """Read the artifact once, verify it against a separately supplied key, return those exact bytes.

    The caller must load the returned bytes, not reopen the path, so the file cannot change between
    verification and use. The lab key is ephemeral; a real device needs a protected trust anchor.
    """
    path = Path(path)
    payload = path.read_bytes()
    manifest = json.loads(sidecar(path, '.manifest.json').read_bytes())
    verify(payload, manifest, sidecar(path, '.sig').read_bytes(), public_key)
    return payload, [
        {'stage': 'ROM / fuse trust anchor', 'status': 'MODELED ONLY'},
        {'stage': 'bootloader / kernel authentication', 'status': 'MODELED ONLY'},
        {'stage': 'signed ONNX artifact + version + device policy', 'status': 'VERIFIED IN USER SPACE'},
        {'stage': 'sensors and mission supervisor', 'status': 'READY'},
    ]


def security_experiments(payload):
    key = Ed25519PrivateKey.generate()
    manifest, signature = sign(payload, key)
    old, old_sig = sign(payload, key, version=1)
    wrong, wrong_sig = sign(payload, key, device='other-board')
    cases = [
        ('valid_update', payload, manifest, signature, key.public_key(), True),
        ('tampered_payload', payload + b'x', manifest, signature, key.public_key(), False),
        ('rollback', payload, old, old_sig, key.public_key(), False),
        ('wrong_device', payload, wrong, wrong_sig, key.public_key(), False),
        ('wrong_signer', payload, manifest, signature, Ed25519PrivateKey.generate().public_key(), False),
        ('modified_manifest', payload, dict(manifest, version=99), signature, key.public_key(), False),
    ]
    results = []
    for name, data, meta, sig, pub, expected in cases:
        try:
            accepted = verify(data, meta, sig, pub)
            reason = 'accepted'
        except (ValueError, InvalidSignature) as error:
            accepted, reason = False, str(error) or 'signature_rejected'
        results.append(
            {'case': name, 'accepted': accepted, 'expected': expected, 'passed': accepted == expected, 'reason': reason}
        )
    return results
