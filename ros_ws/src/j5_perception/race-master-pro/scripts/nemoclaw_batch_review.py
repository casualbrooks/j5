"""Classify prepared temporal strips inside NemoClaw using NVIDIA vision.

This script needs only Python's standard library and NVIDIA_API_KEY. Run it in
the sandbox after uploading the offboard_review.py output directory. It writes
a resumable classified.json beside review.json; no Pi ledger is touched.
"""

import argparse
import base64
import json
import os
import time
from pathlib import Path
from urllib.request import Request, urlopen


def classify(jpeg, racers, model):
    roster = [
        {"id": racer["id"], "name": racer["name"], "color_hint": racer["color"]}
        for racer in racers
    ]
    prompt = (
        "Seven vertically stacked images are chronological frames around one slot-car "
        "finish-line crossing. Track the car across frames. Color hints are weak; blur, "
        "similar colors, shadows, and lane changes can make identity unknowable. "
        f"Racer roster: {json.dumps(roster)}. "
        "Return JSON only with racer_id (one roster ID or null), confidence (0..1), "
        "and reason. Use null unless the entire sequence uniquely supports an ID. "
        "Do not infer a name from a color hint alone."
    )
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": "data:image/jpeg;base64,"
                            + base64.b64encode(jpeg).decode()
                        },
                    },
                ],
            }
        ],
        "temperature": 0.2,
        "max_tokens": 450,
        "stream": False,
        "chat_template_kwargs": {"enable_thinking": False},
    }
    request = Request(
        "https://integrate.api.nvidia.com/v1/chat/completions",
        data=json.dumps(payload).encode(),
        headers={
            "Authorization": "Bearer " + os.environ["NVIDIA_API_KEY"],
            "Content-Type": "application/json",
        },
    )
    with urlopen(request, timeout=120) as response:
        result = json.load(response)
    answer = (result["choices"][0]["message"].get("content") or "").strip()
    if answer.startswith("```"):
        answer = answer.split("\n", 1)[1].rsplit("```", 1)[0]
    try:
        parsed = json.loads(answer)
    except (TypeError, ValueError):
        return {"racer_id": None, "confidence": 0, "reason": "Unusable model response"}
    if parsed.get("racer_id") not in {racer["id"] for racer in racers}:
        parsed["racer_id"] = None
    parsed["confidence"] = max(0.0, min(1.0, float(parsed.get("confidence", 0))))
    return parsed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("review_json", type=Path)
    parser.add_argument("--limit", type=int, default=20)
    parser.add_argument("--model", default="nvidia/nemotron-3.5-super-vl-120b-a12b")
    args = parser.parse_args()
    if not os.environ.get("NVIDIA_API_KEY"):
        parser.error("NVIDIA_API_KEY is required inside the NemoClaw sandbox")
    review = json.loads(args.review_json.read_text())
    output = args.review_json.with_name("classified.json")
    result = (
        json.loads(output.read_text())
        if output.exists()
        else {
            "model": args.model,
            "video_sha256": review["manifest"]["video_sha256"],
            "items": [],
        }
    )
    if result["video_sha256"] != review["manifest"]["video_sha256"]:
        raise ValueError("Existing classifications belong to a different video")
    done = {item["index"] for item in result["items"]}
    remaining = max(0, args.limit)
    for index, event in enumerate(review["events"]):
        if index in done or remaining == 0:
            continue
        jpeg = (args.review_json.parent / event["strip_file"]).read_bytes()
        decision = classify(jpeg, review["manifest"]["racers"], args.model)
        result["items"].append({"index": index, "second": event["second"], **decision})
        temp = output.with_suffix(".partial")
        temp.write_text(json.dumps(result, indent=2))
        temp.replace(output)
        remaining -= 1
        print(
            f"{index + 1}/{len(review['events'])}: {decision['racer_id'] or 'unknown'}"
        )
        time.sleep(0.25)
    print(
        f"Saved {len(result['items'])} of {len(review['events'])} temporal reviews to {output}"
    )


if __name__ == "__main__":
    main()
