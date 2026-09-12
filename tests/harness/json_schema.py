"""A JSON Schema validator covering exactly the keywords protocol/schema.json uses.

Test support, not production code: the codec in ``bridge/protocol.py`` enforces
the catalog's rules directly, so nothing that runs on the rover needs a schema
engine.  What needs one is the promise ``protocol/messages.md`` makes -- that
``schema.json`` is the machine-checkable form of the catalog -- and that
promise is only worth anything if something checks it.

``jsonschema`` is not a dependency of this repo, and adding one for a file that
is read by tests and by whoever writes the next language's client would be a
poor trade.  So this implements the subset that ``schema.json`` actually uses:

    $ref (local pointers only)   oneOf      type        const
    enum                         required   properties  additionalProperties
    minimum

Anything else in a schema is a keyword this validator would silently ignore,
which is why :func:`assert_supported_keywords` fails the moment the schema
grows one.  ``schema.json`` is standard draft 2020-12, so a real validator can
replace this file without touching the schema.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO / "protocol" / "schema.json"
EXAMPLES_DIR = REPO / "protocol" / "examples"

#: Every keyword this validator understands.  Metadata keys are ignored on
#: purpose: they carry no constraint.
SUPPORTED = frozenset(
    {
        "$ref",
        "oneOf",
        "type",
        "const",
        "enum",
        "required",
        "properties",
        "additionalProperties",
        "minimum",
    }
)
METADATA = frozenset({"$schema", "$id", "$defs", "title", "description", "examples"})

_TYPES: dict[str, type | tuple[type, ...]] = {
    "object": dict,
    "array": list,
    "string": str,
    "number": (int, float),
    "integer": int,
    "boolean": bool,
}


class SchemaViolation(Exception):
    """An instance did not satisfy the schema.  Carries the path that failed."""


def load_schema() -> dict[str, Any]:
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def validate(instance: Any, schema: dict[str, Any] | None = None) -> None:
    """Raise :class:`SchemaViolation` unless ``instance`` satisfies the schema."""
    root = schema if schema is not None else load_schema()
    _check(instance, root, root, "$")


def is_valid(instance: Any, schema: dict[str, Any] | None = None) -> bool:
    try:
        validate(instance, schema)
    except SchemaViolation:
        return False
    return True


def assert_supported_keywords(schema: dict[str, Any] | None = None) -> None:
    """Fail if the schema uses a keyword this validator would ignore.

    A validator that ignores what it does not know turns a tightened schema
    into a schema that stopped being checked, without saying so.
    """
    root = schema if schema is not None else load_schema()
    unknown: set[str] = set()

    def walk(node: Any) -> None:
        """Descend through schema positions only.

        The keys of ``properties`` and ``$defs`` are names, not keywords, so
        they are stepped over rather than checked -- a walker that forgets
        that reports every field in the catalog as an unknown keyword.
        """
        if not isinstance(node, dict):
            return
        unknown.update(set(node) - SUPPORTED - METADATA)
        for subschema in node.get("properties", {}).values():
            walk(subschema)
        for subschema in node.get("$defs", {}).values():
            walk(subschema)
        for subschema in node.get("oneOf", []):
            walk(subschema)

    walk(root)
    if unknown:
        raise AssertionError(
            f"protocol/schema.json uses keywords this validator ignores: {sorted(unknown)}"
        )


def _resolve(pointer: str, root: dict[str, Any]) -> dict[str, Any]:
    if not pointer.startswith("#/"):
        raise AssertionError(f"only local $ref pointers are supported, got {pointer!r}")
    node: Any = root
    for part in pointer[2:].split("/"):
        node = node[part]
    return node


def _check(instance: Any, schema: dict[str, Any], root: dict[str, Any], path: str) -> None:
    if "$ref" in schema:
        _check(instance, _resolve(schema["$ref"], root), root, path)
        return

    if "oneOf" in schema:
        matched = [option for option in schema["oneOf"] if is_valid_against(instance, option, root)]
        if len(matched) != 1:
            raise SchemaViolation(
                f"{path}: matched {len(matched)} of the {len(schema['oneOf'])} message "
                f"schemas, expected exactly 1"
            )
        return

    if "type" in schema:
        expected = _TYPES[schema["type"]]
        # bool is an int in Python and is not a number in JSON Schema.
        if isinstance(instance, bool) != (schema["type"] == "boolean"):
            raise SchemaViolation(
                f"{path}: expected {schema['type']}, got {type(instance).__name__}"
            )
        if not isinstance(instance, expected):
            raise SchemaViolation(
                f"{path}: expected {schema['type']}, got {type(instance).__name__}"
            )

    if "const" in schema and instance != schema["const"]:
        raise SchemaViolation(f"{path}: expected {schema['const']!r}, got {instance!r}")

    if "enum" in schema and instance not in schema["enum"]:
        raise SchemaViolation(f"{path}: {instance!r} is not one of {schema['enum']}")

    if "minimum" in schema and isinstance(instance, (int, float)) and instance < schema["minimum"]:
        raise SchemaViolation(f"{path}: {instance} is below the minimum {schema['minimum']}")

    if isinstance(instance, dict):
        for key in schema.get("required", []):
            if key not in instance:
                raise SchemaViolation(f"{path}: missing required property {key!r}")

        properties = schema.get("properties", {})
        for key, value in instance.items():
            if key in properties:
                _check(value, properties[key], root, f"{path}.{key}")
            elif schema.get("additionalProperties") is False:
                raise SchemaViolation(f"{path}: unexpected property {key!r}")


def is_valid_against(instance: Any, schema: dict[str, Any], root: dict[str, Any]) -> bool:
    try:
        _check(instance, schema, root, "$")
    except SchemaViolation:
        return False
    return True
