#!/usr/bin/env python3
"""Interactive Contact Echo Monitor with Class Names in Brackets (§10, §11).

Prints /surface/contacts in real-time, showing class ID alongside human-readable name,
coordinates, confidence, and position uncertainty.

Usage:
  python3 tools/echo_contacts.py             # Monitors /surface/contacts (default)
  python3 tools/echo_contacts.py --onboard   # Monitors /vehicle/contacts
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE_ROOT / "ros2_ws" / "src" / "ps11_common"))

import rclpy
from ps11_common.classes import ClassDatabase
from ps11_interfaces.msg import ContactArray
from rclpy.node import Node

CLASS_ICONS = {
    0: "🗑️  [debris]     ",
    1: "⭐ [starfish]   ",
    2: "🟣 [sea_urchin] ",
    3: "🐚 [scallop]    ",
}


class ContactEchoNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("contact_echo_monitor")
        self.class_db = ClassDatabase()
        self.topic = topic
        self._seen_ids: dict[int, int] = {}  # contact_id -> last_sightings

        self.create_subscription(ContactArray, topic, self._on_contacts, 10)
        print("\n" + "=" * 76)
        print(f"  CONTACT ECHO MONITOR — Subscribed to: {topic}")
        print("=" * 76)
        print(" Class ID Reference:")
        print("   0 -> [debris]      (Defence proxy / man-made target — Priority 1.0)")
        print("   1 -> [starfish]    (Marine life — Priority 0.3)")
        print("   2 -> [sea_urchin]  (Marine life — Priority 0.3)")
        print("   3 -> [scallop]     (Shellfish — Priority 0.2)")
        print("-" * 76)
        print(" Waiting for acoustic contacts to arrive at surface...\n")

    def _on_contacts(self, msg: ContactArray) -> None:
        if not msg.contacts:
            return

        for c in msg.contacts:
            cid = int(c.contact_id)
            sightings = int(c.sightings)

            # Print whenever a new contact appears or sightings update
            if cid not in self._seen_ids or sightings > self._seen_ids[cid]:
                self._seen_ids[cid] = sightings
                try:
                    c_name = self.class_db.id_to_name(c.class_id)
                except KeyError:
                    c_name = f"unknown_{c.class_id}"

                tag = CLASS_ICONS.get(c.class_id, f"[{c_name}]")
                pct = round(c.confidence * 100)

                print("-" * 76)
                print(
                    f" CONTACT #{cid:02d} | Class {c.class_id} {tag} | Conf: {pct:3d}% | Sightings: {sightings}"
                )
                print(
                    f"   Position:    X = {c.position.x:+6.2f} m,  Y = {c.position.y:+6.2f} m,  Depth = {c.depth_m:5.1f} m"
                )
                print(
                    f"   Uncertainty: sigma_xy = ±{c.sigma_xy_m:.1f} m  (Acoustic bucketed resolution)"
                )
                print("-" * 76)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Echo contacts with human-readable class names"
    )
    parser.add_argument(
        "--onboard",
        action="store_true",
        help="Listen to /vehicle/contacts instead of /surface/contacts",
    )
    args = parser.parse_args()

    topic = "/vehicle/contacts" if args.onboard else "/surface/contacts"
    rclpy.init()
    node = ContactEchoNode(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
