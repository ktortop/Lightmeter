import pcbnew, math, sys
from pcbnew import ToMM
from collections import Counter
b=pcbnew.LoadBoard(sys.argv[1])
bb=b.GetBoardEdgesBoundingBox(); print("size mm", round(ToMM(bb.GetWidth()),2), "x", round(ToMM(bb.GetHeight()),2))
ol=pcbnew.SHAPE_POLY_SET(); ok=b.GetBoardPolygonOutlines(ol, False); print("outline closed:", ok, "outlines:", ol.OutlineCount(), "holes in outline:", ol.HoleCount(0))
o=ol.Outline(0); pts=[(o.CPoint(i).x,o.CPoint(i).y) for i in range(o.PointCount())]
segs=[(pts[i],pts[(i+1)%len(pts)]) for i in range(len(pts))]
def seg_d(px,py,ax,ay,bx,by):
    dx,dy=bx-ax,by-ay; L=dx*dx+dy*dy; t=0 if L==0 else max(0,min(1,((px-ax)*dx+(py-ay)*dy)/L)); return math.hypot(px-ax-t*dx,py-ay-t*dy)
def dist_to_outline(poly):
    best=1e18
    for k in range(poly.OutlineCount()):
        oo=poly.Outline(k)
        for j in range(oo.PointCount()):
            p=oo.CPoint(j)
            for (a,c) in segs: best=min(best,seg_d(p.x,p.y,a[0],a[1],c[0],c[1]))
    return best/1e6
cu=[]
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetAttribute()==pcbnew.PAD_ATTRIB_NPTH:
            r=ToMM(p.GetDrillSize().x)/2; c=p.GetPosition(); cu.append((round(min(seg_d(c.x,c.y,a[0],a[1],e[0],e[1]) for a,e in segs)/1e6-r,2),"NPTH "+fp.GetReference())); continue
        ps=pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu, 0, 10000, pcbnew.ERROR_INSIDE)
        cu.append((round(dist_to_outline(ps),2), fp.GetReference()+"."+p.GetNumber()))
for t in b.GetTracks():
    ps=pcbnew.SHAPE_POLY_SET(); t.TransformShapeToPolygon(ps, t.GetLayer() if t.GetClass()=="PCB_TRACK" else pcbnew.F_Cu, 0, 10000, pcbnew.ERROR_INSIDE)
    cu.append((round(dist_to_outline(ps),2), t.GetClass()+" "+t.GetNetname()))
cu.sort(); print("closest copper/holes to edge:", cu[:5])
for z in b.Zones():
    if not z.GetIsRuleArea():
        f=z.GetFilledPolysList(z.GetLayer()); print("zone", z.GetZoneName(), "edge dist", round(dist_to_outline(f),2), "pieces", f.OutlineCount())
a1=b.FindFootprintByReference("A1"); print("Mega NPTH holes:", sorted((round(ToMM(p.GetPosition().x),2),round(ToMM(p.GetPosition().y),2)) for p in a1.Pads() if p.GetAttribute()==pcbnew.PAD_ATTRIB_NPTH))
imu=b.FindFootprintByReference("IMU1"); c=imu.GetPosition()
def mid(t): return (t.GetPosition() if t.GetClass()=="PCB_VIA" else pcbnew.VECTOR2I((t.GetStart().x+t.GetEnd().x)//2,(t.GetStart().y+t.GetEnd().y)//2))
inside=[t.GetNetname() for t in b.GetTracks() if abs(ToMM(mid(t).x-c.x))<6.4 and abs(ToMM(mid(t).y-c.y))<9.9]
print("tracks/vias inside IMU keepout:", dict(Counter(inside)))
pw=[t for t in b.GetTracks() if t.GetClass()=="PCB_TRACK" and t.GetNetname()=="+5V"]
print("nearest +5V track to IMU centre: %.1f mm"%min(math.hypot(ToMM(mid(t).x-c.x),ToMM(mid(t).y-c.y)) for t in pw))
def pad(r,n): return [p for p in b.FindFootprintByReference(r).Pads() if p.GetNumber()==n][0]
def dist(x,y): return round((x.GetPosition()-y.GetPosition()).EuclideanNorm()/1e6,1)
print("decoupling: C3-U1", dist(pad("C3","1"),pad("U1","14")), "| C9-U3", dist(pad("C9","1"),pad("U3","24")), "| C4-SD", dist(pad("C4","1"),pad("J1","4")), "| C2-U2 tab", min(dist(pad("C2","1"),p) for p in b.FindFootprintByReference("U2").Pads() if p.GetNumber()=="2"), "| C1-U2 in", dist(pad("C1","1"),pad("U2","3")), "| R18-U1.9", dist(pad("R18","2"),pad("U1","9")))
for led,(rn,gn) in {"J4 GPS":("LED1_R_MCU","LED1_G_MCU"),"J5 LUX":("LED2_R_MCU","LED2_G_MCU"),"J6 IMU":("LED3_R_MCU","LED3_G_MCU")}.items():
    m={p.GetNetname():p.GetNumber() for p in a1.Pads() if p.GetNetname() in (rn,gn)}
    print(" ",led,"red die <-",m.get(rn)," green die <-",m.get(gn))
ds=b.GetDesignSettings(); print("tent vias:", ds.m_TentViasFront, ds.m_TentViasBack)
thin=sum(1 for d in b.GetDrawings() if d.GetLayer() in (pcbnew.F_SilkS,pcbnew.B_SilkS) and d.GetClass()=="PCB_TEXT" and ToMM(d.GetTextThickness())<0.149)
thin+=sum(1 for fp in b.GetFootprints() for f in fp.GetFields() if f.IsVisible() and f.GetLayer() in (pcbnew.F_SilkS,pcbnew.B_SilkS) and ToMM(f.GetTextThickness())<0.149)
thin+=sum(1 for fp in b.GetFootprints() for g in fp.GraphicalItems() if g.GetLayer() in (pcbnew.F_SilkS,pcbnew.B_SilkS) and g.GetClass()=="PCB_SHAPE" and ToMM(g.GetWidth())<0.149)
print("silk items < 0.15 mm:", thin)
ar=[]
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH:
            ar.append(((min(ToMM(p.GetSize(pcbnew.F_Cu).x),ToMM(p.GetSize(pcbnew.F_Cu).y))-ToMM(p.GetDrillSize().x))/2, fp.GetReference()))
for t in b.GetTracks():
    if t.GetClass()=="PCB_VIA": ar.append(((ToMM(t.GetWidth(pcbnew.F_Cu))-ToMM(t.GetDrillValue()))/2,"via"))
print("min annular ring %.3f mm (%s); min drill %.2f mm; min track %.2f mm" % (min(ar)[0], min(ar)[1], min(ToMM(t.GetDrillValue()) for t in b.GetTracks() if t.GetClass()=="PCB_VIA"), min(ToMM(t.GetWidth()) for t in b.GetTracks() if t.GetClass()=="PCB_TRACK")))
