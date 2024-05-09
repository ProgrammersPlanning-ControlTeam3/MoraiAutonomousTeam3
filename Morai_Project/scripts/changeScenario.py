#!/usr/bin/env python3

import json
import random


def random_boolean(probability):
    return random.random() < probability


def main():
    base_file_path = 'Morai_Project/scenario/random_obstacle.json'

    with open(base_file_path, 'r') as f:
        data = json.load(f)

    object_lists = data['objectList']

    n = 0
    while len(object_lists) > 30:
        if object_lists[n]['DataID'] == 40100019:
            if random_boolean(0.3):
                object_lists.pop(n)
        n += 1
        if n >= len(object_lists):
            n = 0

    with open('src/morai_project/scripts/editScenario/test_data.json', 'w') as f:
        json.dump(data, f, indent=4)


if __name__ == "__main__":
    main()
