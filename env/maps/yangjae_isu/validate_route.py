"""One temporary SUMO-controlled vehicle must finish without teleportation."""
import json
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import sumolib

base = Path(__file__).parent.resolve()
data = json.loads((base/'route.json').read_text(encoding='utf-8'))
with tempfile.TemporaryDirectory(prefix='yangjae-route-') as tmp:
    tmp = Path(tmp)
    routes = ET.Element('routes')
    ET.SubElement(routes, 'route', id='r0', edges=' '.join(data['edges']))
    ET.SubElement(routes, 'vehicle', id='route_check', route='r0', depart='0', departLane='best')
    ET.ElementTree(routes).write(tmp/'check.rou.xml', encoding='utf-8')
    result = subprocess.run([sumolib.checkBinary('sumo'), '-n', str(base/'corridor.net.xml'),
        '-r', str(tmp/'check.rou.xml'), '--end','3600', '--step-length','0.5',
        '--time-to-teleport','-1','--no-step-log','true','--tripinfo-output',str(tmp/'trip.xml')],
        capture_output=True,text=True)
    if result.returncode:
        raise RuntimeError(result.stderr)
    trips = ET.parse(tmp/'trip.xml').findall('tripinfo')
    assert len(trips)==1 and trips[0].get('id')=='route_check', result.stderr
    report = dict(trips[0].attrib)
    report['teleport_disabled'] = True
    report['stderr'] = result.stderr
    (base/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
