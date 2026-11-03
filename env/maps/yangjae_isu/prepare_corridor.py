"""Extract a connected Hyundai Yangjae -> Gangnam Ring -> Sadang -> Isu route."""
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import sumolib

BASE = Path(__file__).parent
print('Reading source network...', flush=True)
net = sumolib.net.readNet(str(BASE/'full.net.xml'))
print('Source edges:', len(net.getEdges()), flush=True)
# Direction-specific OSM edges, verified against road names and geometry.
anchors = [('Hyundai Yangjae / Heolleung-ro', '220199779#2'),
           ('Gangnam Ring westbound', '550118883'),
           ('Seocho Tunnel westbound', '428769853'),
           ('Sadang Station northbound', '521118065#0'),
           ('Isu Station', '215028506')]
route = []
for (_, a), (_, b) in zip(anchors, anchors[1:]):
    path, cost = net.getShortestPath(net.getEdge(a), net.getEdge(b), vClass='passenger')
    if not path:
        raise RuntimeError(f'No passenger route: {a} -> {b}')
    route.extend(path if not route else path[1:])
ids = [e.getID() for e in route]
if len(set(ids)) != len(ids):
    raise RuntimeError('Unexpected route loop')
keep = set(ids)
# Keep a geographical corridor, including opposite carriageways and cross streets.
# Sampling both polylines avoids confusing grade-separated crossings with junctions:
# actual connectivity is still read from the imported network.
radius = 200.0
def samples(shape):
    for a, b in zip(shape, shape[1:]):
        count = max(1, math.ceil(math.dist(a, b)/30))
        for i in range(count):
            yield (a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
    yield shape[-1]
grid = {}
for edge in route:
    for x,y in samples(edge.getShape()):
        grid.setdefault((math.floor(x/radius), math.floor(y/radius)), []).append((x,y))
for edge in net.getEdges():
    for x,y in samples(edge.getShape()):
        gx,gy = math.floor(x/radius), math.floor(y/radius)
        if any((x-px)**2+(y-py)**2 <= radius**2
               for dx in (-1,0,1) for dy in (-1,0,1)
               for px,py in grid.get((gx+dx,gy+dy), [])):
            keep.add(edge.getID())
            break
# Complete junction mouths instead of cutting individual turning connections.
for edge in [net.getEdge(e) for e in list(keep)]:
    for node in (edge.getFromNode(), edge.getToNode()):
        for adjacent in node.getIncoming()+node.getOutgoing():
            if adjacent.getLength() < 500:
                keep.add(adjacent.getID())
(BASE/'keep.txt').write_text('\n'.join(sorted(keep)), encoding='utf-8')
print('Selected corridor edges:', len(keep), flush=True)
subprocess.run([sumolib.checkBinary('netconvert'), '-s', str(BASE/'full.net.xml'),
                '--keep-edges.explicit', ','.join(sorted(keep)),
                '--output.street-names', '-o', str(BASE/'corridor.net.xml')], check=True)
cut = sumolib.net.readNet(str(BASE/'corridor.net.xml'))
for a,b in zip(ids,ids[1:]):
    if cut.getEdge(b) not in cut.getEdge(a).getOutgoing():
        raise RuntimeError(f'Broken connection: {a} -> {b}')
data = {'source':'OpenStreetMap contributors, ODbL 1.0', 'edges':ids,
        'length_m':round(sum(cut.getEdge(e).getLength() for e in ids),2),
        'markers':[{'name':name,'xy':cut.getEdge(e).getShape()[0]} for name,e in anchors]}
(BASE/'route.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
source = {w.get('id'): {t.get('k'):t.get('v') for t in w.findall('tag')}
          for w in ET.parse(BASE/'source.osm.xml').getroot().findall('way')}
audit = []
for edge in cut.getEdges():
    tags = source.get(edge.getID().lstrip('-').split('#')[0], {})
    audit.append({'edge':edge.getID(), 'road':edge.getName(),
                  'on_route':edge.getID() in ids, 'lanes':edge.getLaneNumber(),
                  'lane_count_in_osm': any(k in tags for k in ('lanes','lanes:forward','lanes:backward')),
                  'source_tags': {k:v for k,v in tags.items() if k in ('lanes','lanes:forward','lanes:backward','turn:lanes','bus:lanes','lanes:bus','maxspeed','oneway')},
                  'outgoing': {e.getID(): [{'from_lane':c.getFromLane().getIndex(),
                               'to_lane':c.getToLane().getIndex()} for c in connections]
                               for e,connections in edge.getOutgoing().items()}})
report = {'corridor_radius_m':radius, 'edges':len(audit),
          'route_edges_with_unverified_lane_count':sum(r['on_route'] and not r['lane_count_in_osm'] for r in audit),
          'note':'OSM tags are evidence, not a field survey. Missing lane counts remain defaults.',
          'roads':audit}
(BASE/'lane_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Route:',len(ids),'edges,',data['length_m'],'m; corridor:',len(cut.getEdges()),'edges')
print([(e.getID(), e.getName()) for e in route])
