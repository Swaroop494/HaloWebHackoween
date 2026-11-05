"""Downloader for KenneyNL assets for PyBullet compatibility."""
from __future__ import annotations

import os
import sys
import json
import requests
from pathlib import Path
from typing import Optional
from dataclasses import dataclass


@dataclass
class AssetPack:
    """Represents a KenneyNL asset pack."""
    name: str
    url: str
    format: str
    category: str
    download_url: Optional[str] = None


KENNEY_PACKS = [
    AssetPack(
        name="Factory Kit",
        url="https://kenney.nl/assets/factory-kit",
        format="OBJ, FBX, GLB",
        category="industrial",
    ),
    AssetPack(
        name="Industrial Pack",
        url="https://kenney.nl/assets/industrial",
        format="OBJ, FBX, GLB",
        category="industrial",
    ),
    AssetPack(
        name="Interior Pack",
        url="https://kenney.nl/assets/interior",
        format="OBJ, FBX, GLB",
        category="interior",
    ),
    AssetPack(
        name="Architectural Pack",
        url="https://kenney.nl/assets/architectural",
        format="OBJ, FBX, GLB",
        category="architectural",
    ),
]


def get_asset_download_url(pack: AssetPack) -> Optional[str]:
    """Get the actual download URL for a KenneyNL asset pack."""
    # KenneyNL assets are hosted on itch.io
    # Extract pack name from URL
    pack_name = pack.url.split("/")[-1].replace("-", "_")
    return f"https://kenney-assets.itch.io/{pack_name}"


def download_asset_pack(pack: AssetPack, output_dir: Path) -> bool:
    """Download an asset pack from KenneyNL."""
    print(f"Downloading {pack.name}...")

    try:
        download_url = get_asset_download_url(pack)
        response = requests.get(download_url, timeout=30)
        response.raise_for_status()

        # Save the response (this is a simplified version)
        # In practice, you'd want to handle the actual file download
        output_file = output_dir / f"{pack.name.replace(' ', '_')}.html"
        output_file.write_text(response.text)

        print(f"Downloaded {pack.name} to {output_file}")
        return True

    except requests.RequestException as e:
        print(f"Error downloading {pack.name}: {e}")
        return False


def list_available_assets() -> list[AssetPack]:
    """List all available KenneyNL asset packs for PyBullet."""
    return KENNEY_PACKS


def find_pybullet_compatible_assets() -> list[AssetPack]:
    """Find asset packs that are PyBullet-compatible."""
    compatible = []
    for pack in KENNEY_PACKS:
        if "OBJ" in pack.format or "FBX" in pack.format:
            compatible.append(pack)
    return compatible


def main():
    """Main entry point for asset downloader."""
    if len(sys.argv) < 2:
        print("KenneyNL Asset Downloader")
        print("=" * 40)
        print("\nAvailable asset packs:")
        for i, pack in enumerate(KENNEY_PACKS, 1):
            print(f"{i}. {pack.name}")
            print(f"   Category: {pack.category}")
            print(f"   Formats: {pack.format}")
            print()

        print("\nUsage:")
        print("  python kenney_downloader.py list")
        print("  python kenney_downloader.py download <pack_index>")
        print("  python kenney_downloader.py download_all")
        return

    command = sys.argv[1].lower()

    if command == "list":
        print("Available KenneyNL Asset Packs:")
        print("=" * 50)
        for i, pack in enumerate(KENNEY_PACKS, 1):
            print(f"{i}. {pack.name}")
            print(f"   URL: {pack.url}")
            print(f"   Category: {pack.category}")
            print(f"   Formats: {pack.format}")
            print()

    elif command == "download":
        if len(sys.argv) < 3:
            print("Error: Please specify a pack index")
            print("Usage: python kenney_downloader.py download <pack_index>")
            return

        try:
            pack_index = int(sys.argv[2]) - 1
            if pack_index < 0 or pack_index >= len(KENNEY_PACKS):
                print(f"Error: Invalid pack index. Must be 1-{len(KENNEY_PACKS)}")
                return

            pack = KENNEY_PACKS[pack_index]
            output_dir = Path(__file__).parent / "downloads" / pack.name.replace(" ", "_")
            output_dir.mkdir(parents=True, exist_ok=True)

            download_asset_pack(pack, output_dir)

        except ValueError:
            print("Error: Pack index must be a number")

    elif command == "download_all":
        output_base = Path(__file__).parent / "downloads"
        output_base.mkdir(parents=True, exist_ok=True)

        success_count = 0
        for pack in KENNEY_PACKS:
            output_dir = output_base / pack.name.replace(" ", "_")
            if download_asset_pack(pack, output_dir):
                success_count += 1

        print(f"\nDownloaded {success_count}/{len(KENNEY_PACKS)} packs")

    else:
        print(f"Unknown command: {command}")
        print("\nUsage:")
        print("  python kenney_downloader.py list")
        print("  python kenney_downloader.py download <pack_index>")
        print("  python kenney_downloader.py download_all")


if __name__ == "__main__":
    main()
