from pathlib import Path
import sumolib
import traci
net = sumolib.net.readNet(str(Path(__file__).parent/'full.net.xml'))
traci.start([sumolib.checkBinary('sumo'), '-n', str(Path(__file__).parent/'full.net.xml'), '--no-step-log'])
for name, lon, lat in [('start',127.042,37.463),('yangjae',127.036,37.463),('seonam',127.018,37.457),('tunnel',127.005,37.455),('sadangIC',126.987,37.464),('sadang',126.982,37.477),('isu',126.982,37.4865)]:
    x,y = traci.simulation.convertGeo(lon,lat,fromGeo=True)
    candidates = sorted(net.getNeighboringEdges(x,y,400), key=lambda v:v[1])
    print(name, [(e.getID(), e.getName(),round(d), e.getLaneNumber(), tuple(round(z) for z in e.getShape()[0]),tuple(round(z) for z in e.getShape()[-1])) for e,d in candidates[:8]])
traci.close()
