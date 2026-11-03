"""Download small OSM API tiles; merge by object ID. Run only to refresh source."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import urlopen, Request
import xml.etree.ElementTree as ET

BASE = Path(__file__).parent
def tile(box):
    name = BASE / ('tile_' + '_'.join(map(str, box)) + '.xml')
    if not name.exists():
        url = 'https://api.openstreetmap.org/api/0.6/map?bbox=' + ','.join(map(str, box))
        with urlopen(Request(url, headers={'User-Agent': 'SUMO-course-road-builder/1.0'}), timeout=90) as r:
            name.write_bytes(r.read())
    return ET.parse(name).getroot()

if __name__ == '__main__':
    boxes = [(round(126.97+i*.02, 4), round(37.435+j*.02, 4),
              round(126.99+i*.02, 4), round(37.455+j*.02, 4))
             for i in range(4) for j in range(3)]
    objects = {}
    with ThreadPoolExecutor(max_workers=2) as pool:
        for root in pool.map(tile, boxes):
            for elem in root:
                if elem.tag in ('node', 'way', 'relation'):
                    objects[(elem.tag, elem.get('id'))] = elem
            print('Downloaded tile', flush=True)
    root = ET.Element('osm', version='0.6', generator='SUMO coursework; OpenStreetMap contributors')
    for kind in ('node', 'way', 'relation'):
        root.extend(e for (k, _), e in objects.items() if k == kind)
    ET.ElementTree(root).write(BASE/'source.osm.xml', encoding='utf-8', xml_declaration=True)
    print('Saved', len(objects), 'objects')
