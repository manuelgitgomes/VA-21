function TT = transform2D(Tin)
x = Tin(1);
y = Tin(2);
ang = Tin(3);
TT = [ cos(ang) -sin(ang) x
       sin(ang)  cos(ang) y
      0          0        1];