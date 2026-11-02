from reportlab.lib.pagesizes import A4, landscape
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
T={'0':0x034,'1':0x121,'2':0x061,'3':0x160,'4':0x031,'5':0x130,'6':0x070,'7':0x025,'8':0x124,'9':0x064,'A':0x109,'B':0x049,'C':0x148,'D':0x019,'E':0x118,'F':0x058,'G':0x00D,'H':0x10C,'I':0x04C,'J':0x01C,'K':0x103,'L':0x043,'M':0x142,'N':0x013,'O':0x112,'P':0x052,'Q':0x007,'R':0x106,'S':0x046,'T':0x016,'U':0x181,'V':0x0C1,'W':0x1C0,'X':0x091,'Y':0x190,'Z':0x0D0,'-':0x085,'.':0x184,'$':0x0A8,'/':0x0A2,'+':0x08A,'%':0x02A,'*':0x094}
NARROW, WIDE = 4.0, 10.0          # mm  (wide = 2.5 x narrow)
def widths(p):
    out=[]
    syms='*'+p+'*'
    for k,ch in enumerate(syms):
        pat=T[ch]
        out+= [WIDE if (pat>>(8-i))&1 else NARROW for i in range(9)]
        if k<len(syms)-1: out.append(NARROW)
    return out
CHARS=list("ABCDEFLRU17X")
W,H=landscape(A4); c=canvas.Canvas("code39_test_sheet.pdf",pagesize=(W,H))
c.setTitle("Code 39 test barcodes - INF2004 subsystem 3")
per_page=4; bar_h=32*mm; quiet=15*mm
open("widths.txt","w").write("".join(ch+" "+" ".join(str(int(x*1000)) for x in widths(ch))+"\n" for ch in CHARS))
for i,ch in enumerate(CHARS):
    if i%per_page==0:
        if i: c.showPage()
        c.setFont("Helvetica-Bold",13); c.drawString(15*mm,H-12*mm,"Code 39 test barcodes  -  INF2004 subsystem 3")
        c.setFont("Helvetica",8.5)
        c.drawString(15*mm,H-17*mm,f"Print at 100% / Actual size (no 'fit to page'). Narrow = {NARROW:g} mm, wide = {WIDE:g} mm. Matte paper, laser printer preferred (some inkjet black is invisible to IR).")
        c.drawString(15*mm,H-21*mm,"Check: the ruler below must measure exactly 100 mm. Keep the white margin left and right of each barcode (quiet zone).")
        y0=H-26*mm; c.setLineWidth(0.6); c.line(15*mm,y0,115*mm,y0)
        for t in range(0,101,10):
            c.line(15*mm+t*mm,y0,15*mm+t*mm,y0-(3 if t%50 else 5)*mm)
        c.drawString(117*mm,y0-2*mm,"100 mm")
    slot=i%per_page; top=H-38*mm-slot*(bar_h+12*mm)
    w=widths(ch); total=sum(w)*mm; x=(W-total)/2
    c.setFillGray(0)
    for k,ww in enumerate(w):
        if k%2==0: c.rect(x,top-bar_h,ww*mm,bar_h,stroke=0,fill=1)
        x+=ww*mm
    c.setFont("Helvetica-Bold",11); c.drawCentredString(W/2,top-bar_h-6*mm,f'"{ch}"   (*{ch}*,  {total/mm:.0f} mm wide)')
    c.setStrokeGray(0.75); c.setLineWidth(0.3)
    c.rect((W-total)/2-quiet,top-bar_h-1*mm,total+2*quiet,bar_h+2*mm,stroke=1,fill=0); c.setStrokeGray(0)
c.save()
