using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using Inventor;
using File=System.IO.File;

public partial class RobotCad {
 static readonly string[] linkNames={"BASE_SUBASSEMBLY","HOMBRO_SUBASSEMBLY","BRAZO_SUBASSEMBLY","ANTEBRAZO_SUBASSEMBLY","MUNECA_J4_SUBASSEMBLY","MUNECA_J5_SUBASSEMBLY","BRIDA_J6_SUBASSEMBLY"};
 static double[] offsetsX={0,0,160,130,45,104};
 static double[] offsetsZ={178,95,0,0,0,0};
 static string[] axes={"Z","Y","Y","X","Y","X"};
 static double[] mins={-160,-95,-110,-150,-70,-160},maxs={160,5,110,150,70,160};
 static string Part(string name,Action<PartDocument> make,double density=2.7,string dir="03_Piezas",double mass=-1,int r=45,int g=113,int b=140){
  string fn=P(dir,name+".ipt");if(File.Exists(fn))return fn;
  var d=NewPart(name,density);make(d);Color(d,r,g,b);return Save(d,name,dir,mass);
 }
 static string Ring(string name,string axis,double x,double y,double z,double od,double id,double width,double density=7.85,string dir="03_Piezas"){
  return Part(name,d=>Round(d,"SECCION",axis,x,y,z,od,id,width),density,dir,-1,178,184,190);
 }
 static string Plate(string name,string axis,double x,double y,double z,double a,double b,double t,double hole=0){return Part(name,d=>{Rect(d,"PLACA",axis,x,y,z,a,b,t);if(hole>0)Round(d,"ALOJAMIENTO",axis,x,y,z,hole,0,t,true);});}
 static string Pulley(string name,string axis,double x,double y,double z,int teeth,double bore,double width){
  double od=teeth*3/Math.PI;
  return Part(name,d=>{Round(d,"ENVOLVENTE_PASO_HTD3",axis,x,y,z,od,bore,width);},2.7,"03_Piezas",-1,208,145,40);
 }
 // Belt envelope follows the external tangents of two pitch circles. Tooth
 // geometry is deliberately omitted; the BOM identifies these as layout models.
 static string Belt(string name,string axis,double x,double y,double z,double au,double av,int n1,int n2,double width){
  return Part(name,d=>{
   var s=Sketch(d,"ENVOLVENTE_CORREA",axis,x,y,z);
   double r1=n1*3/(2*Math.PI),r2=n2*3/(2*Math.PI),c=Math.Sqrt(au*au+av*av),phi=Math.Atan2(av,au),beta=Math.Acos((r1-r2)/c);
   foreach(double add in new[]{1.5,0.15}){
    var points=new List<Point2d>();
    for(int k=0;k<=36;k++){double th=phi+beta+(2*Math.PI-2*beta)*k/36;points.Add(Q(s,axis,x,y,z,(r1+add)*Math.Cos(th),(r1+add)*Math.Sin(th)));}
    for(int k=0;k<=36;k++){double th=phi-beta+2*beta*k/36;points.Add(Q(s,axis,x,y,z,au+(r2+add)*Math.Cos(th),av+(r2+add)*Math.Sin(th)));}
    var first=s.SketchLines.AddByTwoPoints(points[0],points[1]);var previous=first;for(int k=2;k<points.Count;k++)previous=s.SketchLines.AddByTwoPoints(previous.EndSketchPoint,points[k]);s.SketchLines.AddByTwoPoints(previous.EndSketchPoint,first.StartSketchPoint);
   }Extrude(d,s,width);
  },1.2,"03_Piezas",-1,42,45,50);
 }
 static string Bearing(string name,string axis,double x,double y,double z,int type){double id=type==61804?20:type==6001?12:10;double od=type==61804?32:type==6001?28:26;double w=type==61804?7:8;
  return Part(name,d=>{Round(d,"CONTORNO_CATALOGO",axis,x,y,z,od,id,w);},7.85,"02_Componentes_Comerciales",-1,105,113,125);
 }
 static void Motor(AssemblyDocument d,string name,bool big,string axis,double x,double y,double z){
  var o=Add(d,P("02_Componentes_Comerciales",big?"MOTOR_GOBILDA_5203_71.ipt":"MOTOR_POLOLU_4845.ipt"),name);
  UnitVector u,v;Ax(axis,out u,out v);var n=u.CrossProduct(v);var m=tg.CreateMatrix();
  double[,] r=new double[3,3];
  double[] uu={u.X,u.Y,u.Z}, vv={v.X,v.Y,v.Z},nn={n.X,n.Y,n.Z};
  double[] src=big?new[]{-37.1707276942743,110.487452318783,-11.0784291472375}:new[]{0.0,0.0,0.0};
  for(int i=0;i<3;i++){r[i,0]=uu[i];r[i,1]=big?nn[i]:vv[i];r[i,2]=big?-vv[i]:nn[i];}
  double[] pos={x,y,z};for(int i=0;i<3;i++){for(int j=0;j<3;j++)m.Cell[i+1,j+1]=r[i,j];m.Cell[i+1,4]=(pos[i]-r[i,0]*src[0]-r[i,1]*src[1]-r[i,2]*src[2])/10;}
  o.Transformation=m;o.Grounded=true;
 }
 static string Skeleton(int i){return Part("REFERENCIAS_L"+i,d=>{
  if(i>0)Frame(d,"ENTRADA", "-"+axes[i-1],0,0,0);
  if(i<6)Frame(d,"SALIDA",axes[i],offsetsX[i],0,offsetsZ[i]);
 },2.7,"03_Piezas");}
 static void Bolt(AssemblyDocument a,string label,string axis,double x,double y,double z,double dia,double length){
  var f=Part("TORNILLO_REF_"+axis+"_"+dia+"_"+length,d=>{Round(d,"VASTAGO",axis,0,0,0,dia,0,length);Round(d,"CABEZA",axis,0,0,-(axis=="Z"?dia:0),dia*1.7,0,dia);},7.85,"02_Componentes_Comerciales",-1,72,77,84);
  Add(a,f,label,x,y,z);
 }
 static void BuildRobot(){
  var af=new string[7];
  for(int i=0;i<7;i++){
   var a=NewAsm(linkNames[i]);var sk=Add(a,Skeleton(i),"REFERENCIAS");sk.BOMStructure=BOMStructureEnum.kReferenceBOMStructure;sk.Visible=false;
   if(i==0)BuildBase(a);if(i==1)BuildShoulder(a);if(i==2)BuildUpper(a);if(i==3)BuildFore(a);if(i==4)BuildRoll(a);if(i==5)BuildPitch(a);if(i==6)BuildFlange(a);
   af[i]=SaveAsm(a,linkNames[i]);
  }
  Assemble(af);
 }
 static void BuildBase(AssemblyDocument a){
  Add(a,Part("BASE_001",d=>{Rect(d,"PLACA_8","Z",-15,0,0,240,210,8);Holes(d,"ANCLAJES_M8","Z",-15,0,0,new double[,]{{-100,-85},{100,-85},{100,85},{-100,85}},9,8);Holes(d,"PEDESTAL_M5","Z",0,0,0,new double[,]{{-28,-28},{28,-28},{28,28},{-28,28}},5.5,8);}),"BASE");
  Add(a,Ring("PEDESTAL_J1_001","Z",0,0,8,55,36,124,2.7),"PEDESTAL");
  foreach(double z in new[]{132.0,160.0}){
   Add(a,Part("PORTARODAMIENTO_J1_"+z,d=>{Rect(d,"PLACA","Z",0,0,z,74,74,8);Round(d,"ASIENTO_32","Z",0,0,z,32,0,8,true);Holes(d,"M5","Z",0,0,z,new double[,]{{-28,-28},{28,-28},{28,28},{-28,28}},5.5,8);}),"PORTARODAMIENTO_"+z);
   Add(a,Bearing("ROD_61804_J1_"+z,"Z",0,0,z==132?133:160,61804),"ROD_J1_"+z);
  }
  foreach(double x in new[]{-28.0,28.0})foreach(double y in new[]{-28.0,28.0})Add(a,Ring("ESPACIADOR_J1_"+x+"_"+y,"Z",x,y,140,9,5.5,20,2.7),"ESPACIADOR_"+x+"_"+y);
  Motor(a,"MOTOR_J1",true,"Z",-76,0,134);
  Add(a,Part("SOPORTE_MOTOR_J1",d=>{Rect(d,"MESA","Z",-76,0,134,54,56,5);Round(d,"EJE","Z",-76,0,134,12,0,5,true);Holes(d,"M4","Z",-76,0,134,new double[,]{{-8,-8},{-8,8},{8,-8},{8,8}},4.5,5);Rect(d,"PATA_A","Z",-100,0,8,6,56,126);}),"SOPORTE_MOTOR");
  Add(a,Pulley("POLEA_J1_20T","Z",-76,0,145,20,8,9),"POLEA_MOTRIZ_J1");
  Add(a,Belt("CORREA_J1_ENV","Z",-76,0,145,76,0,20,60,9),"CORREA_J1");
 }
 static void BuildShoulder(AssemblyDocument a){
  Add(a,Ring("EJE_J1_001","Z",0,0,-50,20,12,50),"EJE_J1");
  Add(a,Pulley("POLEA_J1_60T","Z",0,0,-33,60,20,9),"POLEA_CONDUCIDA_J1");
  Add(a,Part("PLATAFORMA_J1_001",d=>{Round(d,"DISCO","Z",0,0,0,100,12,8);Holes(d,"FIJACION","Z",0,0,0,new double[,]{{-28,-28},{28,-28},{28,28},{-28,28}},5.5,8);}),"PLATAFORMA");
  foreach(double y in new[]{-44.0,36.0}){
   Add(a,Part("SOPORTE_J2_"+y,d=>{Rect(d,"MEJILLA","Y",0,y,52,102,76,8);Round(d,"ASIENTO_6001","Y",0,y,95,28,0,8,true);Holes(d,"TORNILLOS_BASE","Y",0,y,15,new double[,]{{0,-25},{0,25}},5.5,8);}),"MEJILLA_J2_"+y);
   Add(a,Bearing("ROD_6001_J2_"+y,"Y",0,y,95,6001),"ROD_J2_"+y);
  }
  // J2 5:1. A separate 12 mm input shaft carries the belt load.
  Motor(a,"MOTOR_J2",true,"Y",-78,-108,55);
  Add(a,Part("SOPORTE_MOTOR_J2",d=>{Rect(d,"CARA","Y",-78,-108,55,58,56,6);Round(d,"PASO","Y",-78,-108,55,12,0,6,true);Holes(d,"M4","Y",-78,-108,55,new double[,]{{-8,-8},{-8,8},{8,-8},{8,8}},4.5,6);Rect(d,"PUENTE","Y",-78,-102,29,6,56,64);Rect(d,"UNION_HOMBRO","Y",-43,-44,29,12,50,8);}),"SOPORTE_MOTOR_J2");
  Add(a,Ring("ACOPLE_J2_REX_ENV","Y",-78,-90,55,20,8,20,2.7),"ACOPLE_J2_PENDIENTE_REX");
  Add(a,Ring("EJE_ENTRADA_J2","Y",-78,-80,55,12,0,52),"EJE_ENTRADA_J2");
  foreach(double y in new[]{-70.0,-40.0}){
   Add(a,Part("APOYO_ENTRADA_J2_"+y,d=>{Rect(d,"SOPORTE","Y",-78,y,51,46,42,8);Round(d,"ASIENTO","Y",-78,y,55,28,0,8,true);}),"APOYO_ENTRADA_"+y);
   Add(a,Bearing("ROD_ENTRADA_J2_"+y,"Y",-78,y,55,6001),"ROD_ENTRADA_"+y);
  }
  Add(a,Pulley("POLEA_J2_20T","Y",-78,-60,55,20,12,15),"POLEA_MOTRIZ_J2");
  Add(a,Belt("CORREA_J2_ENV","Y",-78,-60,55,40,78,20,100,15),"CORREA_J2");
 }
 static void BuildUpper(AssemblyDocument a){
  Add(a,Ring("EJE_J2_001","Y",0,-66,0,12,0,114),"EJE_J2");
  Add(a,Pulley("POLEA_J2_100T","Y",0,-60,0,100,12,15),"POLEA_J2");
  foreach(double y in new[]{-32.0,28.0}){
   Add(a,Part("BRAZO_SUPERIOR_"+y,d=>{Rect(d,"ALMA","Y",80,y,0,42,160,4);Round(d,"CABEZA_J2","Y",0,y,0,42,0,4);Round(d,"CABEZA_J3","Y",160,y,0,42,0,4);Holes(d,"EJES","Y",0,y,0,new double[,]{{0,0},{0,160}},12,4);Rect(d,"ALIGERADO","Y",91,y,0,22,82,4,true);}),"BRAZO_PLACA_"+y);
   Add(a,Ring("CUBO_J2_"+y,"Y",0,y<0?-35:23,0,24,12,8,2.7),"CUBO_"+y);
  }
  foreach(double x in new[]{22.0,138.0})Add(a,Plate("TRAVESANO_BRAZO_"+x,"Y",x,-28,0,32,8,56),"TRAVESANO_"+x);
  // Outboard motor leaves the plate spacing available for cables and shafts.
  Motor(a,"MOTOR_J3",true,"-Y",38,103,0);
  Add(a,Part("SOPORTE_MOTOR_J3",d=>{Rect(d,"CARA","Y",38,97,0,54,54,6);Round(d,"PASO","Y",38,97,0,12,0,6,true);Holes(d,"M4","Y",38,97,0,new double[,]{{-8,-8},{-8,8},{8,-8},{8,8}},4.5,6);Rect(d,"PUENTE","Y",38,32,-24,6,54,65);}),"SOPORTE_MOTOR_J3");
  Add(a,Ring("ACOPLE_J3_REX_ENV","Y",38,66,0,20,8,20,2.7),"ACOPLE_J3_PENDIENTE_REX");
  Add(a,Ring("EJE_ENTRADA_J3","Y",38,30,0,12,0,47),"EJE_ENTRADA_J3");
  foreach(double y in new[]{32.0,62.0}){
   Add(a,Plate("APOYO_ENTRADA_J3_"+y,"Y",38,y,-3,42,38,8,28),"APOYO_ENTRADA_J3_"+y);
   Add(a,Bearing("ROD_ENTRADA_J3_"+y,"Y",38,y,0,6001),"ROD_ENTRADA_J3_"+y);
  }
  Add(a,Pulley("POLEA_J3_20T","Y",38,43,0,20,12,15),"POLEA_MOTRIZ_J3");
  Add(a,Belt("CORREA_J3_ENV","Y",38,43,0,0,122,20,60,15),"CORREA_J3");
  foreach(double y in new[]{-32.0,24.0}){
   Add(a,Ring("PORTARODAMIENTO_J3_"+y,"Y",160,y,0,38,28,8,2.7),"PORTARODAMIENTO_J3_"+y);
   Add(a,Bearing("ROD_6001_J3_"+y,"Y",160,y,0,6001),"ROD_J3_"+y);
  }
 }
 static void BuildFore(AssemblyDocument a){
  Add(a,Ring("EJE_J3_001","Y",0,-38,0,12,0,102),"EJE_J3");
  Add(a,Pulley("POLEA_J3_60T","Y",0,43,0,60,12,15),"POLEA_J3");
  foreach(double y in new[]{-22.0,18.0}){
   Add(a,Part("ANTEBRAZO_"+y,d=>{Rect(d,"ALMA","Y",66,y,0,34,132,4);Round(d,"CODO","Y",0,y,0,36,0,4);Round(d,"EJE","Y",0,y,0,12,0,4,true);Rect(d,"ALIGERADO","Y",65,y,0,18,80,4,true);}),"PLACA_ANTEBRAZO_"+y);
  }
  Add(a,Plate("TRAVESANO_ANTEBRAZO","Y",18,-18,0,26,8,36),"TRAVESANO");
  foreach(double x in new[]{104.0,133.0}){
   Add(a,Part("SOPORTE_J4_"+x,d=>{Rect(d,"PLACA","X",x,0,15,52,72,7);Round(d,"ALOJAMIENTO_61804","X",x,0,0,32,0,7,true);if(x==104){Round(d,"MOTOR","X",x,0,48,7.5,0,7,true);Holes(d,"M3","X",x,0,48,new double[,]{{-8.5,0},{8.5,0}},3.4,7);}}),"SOPORTE_J4_"+x);
   Add(a,Bearing("ROD_61804_J4_"+x,"X",x,0,0,61804),"ROD_J4_"+x);
  }
  Motor(a,"MOTOR_J4",false,"X",104,0,48);
  Add(a,Pulley("POLEA_J4_20T","X",112,0,48,20,4,9),"POLEA_MOTRIZ_J4");
  Add(a,Belt("CORREA_J4_ENV","X",112,0,48,0,-48,20,60,9),"CORREA_J4");
 }
 static void BuildRoll(AssemblyDocument a){
  Add(a,Ring("EJE_HUECO_J4_001","X",-40,0,0,20,12,47),"EJE_J4");
  Add(a,Pulley("POLEA_J4_60T","X",-28,0,0,60,20,9),"POLEA_J4");
  Add(a,Plate("MUNECA_J4_TRAVESANO","X",7,0,0,78,40,5,12),"TRAVESANO_J4");
  foreach(double y in new[]{-39.0,34.0}){
   Add(a,Part("HORQUILLA_J5_"+y,d=>{Rect(d,"MEJILLA","Y",23,y,0,40,34,5);Round(d,"CABEZA","Y",35,y,0,40,0,5);Round(d,"ASIENTO_6000","Y",35,y,0,26,0,5,true);}),"HORQUILLA_J5_"+y);
   Add(a,Bearing("ROD_6000_J5_"+y,"Y",35,y<0?-39:31,0,6000),"ROD_J5_"+y);
  }
  Motor(a,"MOTOR_J5",false,"Y",-15,-69,0);
  Add(a,Part("SOPORTE_MOTOR_J5",d=>{Rect(d,"CARA","Y",-15,-69,0,38,36,4);Round(d,"RESALTE","Y",-15,-69,0,7.5,0,4,true);Holes(d,"M3","Y",-15,-69,0,new double[,]{{-8.5,0},{8.5,0}},3.4,4);Rect(d,"PUENTE","Y",-15,-65,-17,4,36,26);Rect(d,"BRAZO","Y",-1,-39,-17,4,64,5);}),"SOPORTE_MOTOR_J5");
  Add(a,Pulley("POLEA_J5_20T","Y",-15,-62,0,20,4,9),"POLEA_MOTRIZ_J5");
  Add(a,Belt("CORREA_J5_ENV","Y",-15,-62,0,0,50,20,60,9),"CORREA_J5");
 }
 static void BuildPitch(AssemblyDocument a){
  Add(a,Ring("EJE_J5_001","Y",0,-66,0,10,0,110),"EJE_J5");
  Add(a,Pulley("POLEA_J5_60T","Y",0,-62,0,60,10,9),"POLEA_J5");
  foreach(double y in new[]{-29.0,26.0}){
   Add(a,Part("MUNECA_J5_"+y,d=>{Rect(d,"MEJILLA","Y",50,y,0,30,100,3);Round(d,"PIVOTE","Y",0,y,0,30,0,3);Round(d,"EJE","Y",0,y,0,10,0,3,true);Rect(d,"ALIGERADO","Y",40,y,0,16,54,3,true);}),"MEJILLA_MUNECA_"+y);
  }
  Motor(a,"MOTOR_J6",false,"X",73,0,0);
  Add(a,Part("SOPORTE_MOTOR_J6",d=>{Rect(d,"CARA","X",73,0,0,52,34,4);Round(d,"RESALTE","X",73,0,0,7.5,0,4,true);Holes(d,"M3","X",73,0,0,new double[,]{{-8.5,0},{8.5,0}},3.4,4);}),"SOPORTE_MOTOR_J6");
  Add(a,Part("PORTARODAMIENTO_J6",d=>{Rect(d,"CARA","X",96,0,0,52,36,8);Round(d,"ASIENTO_6000","X",96,0,0,26,0,8,true);}),"PORTARODAMIENTO_J6");
  Add(a,Bearing("ROD_6000_J6","X",96,0,0,6000),"ROD_J6");
 }
 static void BuildFlange(AssemblyDocument a){
  Add(a,Ring("EJE_J6_001","X",-17,0,0,10,0,21),"EJE_J6");
  Add(a,Part("ACOPLE_J6_4_10",d=>{Round(d,"CUERPO","X",-25,0,0,18,0,17);Round(d,"TALADRO_4","X",-25,0,0,4,0,8,true);Round(d,"TALADRO_10","X",-17,0,0,10,0,9,true);}),"ACOPLE_J6");
  Add(a,Part("BRIDA_EFECTOR_001",d=>{Round(d,"BRIDA","X",4,0,0,40,10,5);Holes(d,"PATRON_M3","X",4,0,0,new double[,]{{14,0},{-14,0},{0,14},{0,-14}},3.4,5);}),"BRIDA_EFECTOR");
 }
 static object Marker(ComponentOccurrence link,string which,bool circle){var a=(AssemblyComponentDefinition)link.Definition;var sk=a.Occurrences[1];var pd=(PartComponentDefinition)sk.Definition;var s=pd.Sketches[which];return InventorApiHelpers.Proxy(link,InventorApiHelpers.Proxy(sk,circle?(object)s.SketchCircles[1]:(object)s.SketchLines[1]));}
 static void Assemble(string[] af){
  var d=NewAsm("ROBOT_6DOF");var occ=new ComponentOccurrence[7];double x=0,z=0;
  for(int i=0;i<7;i++){if(i>0){x+=offsetsX[i-1];z+=offsetsZ[i-1];}occ[i]=Add(d,af[i],"L"+i+"_"+linkNames[i],x,0,z);}
  for(int i=0;i<6;i++){
   var j=InventorApiHelpers.AddRevolute(d,"J"+(i+1)+"_REVOLUTA",Marker(occ[i+1],"ENTRADA",true),Marker(occ[i],"SALIDA",true),Marker(occ[i+1],"ENTRADA",false),Marker(occ[i],"SALIDA",false),mins[i],maxs[i],0,false);
   Log("JOINT J"+(i+1)+" "+j.HealthStatus);
  }
  occ[0].Grounded=true;d.Update2(true);SaveAsm(d,"ROBOT_6DOF");
  d=(AssemblyDocument)app.Documents.Open(P("04_Ensamblajes","ROBOT_6DOF.iam"),true);
  var cam=app.ActiveView.Camera;cam.ViewOrientationType=ViewOrientationTypeEnum.kIsoTopRightViewOrientation;cam.Apply();app.ActiveView.Fit();
  ExportMass(d,"HORIZONTAL");InventorApiHelpers.ExportPng(app,P("00_Documentacion","ROBOT_HORIZONTAL.png"),1800,1300);d.Save();
 }
 static void ExportMass(AssemblyDocument d,string pose){
  var rows=new List<string>{"link,name,mass_kg,cx_mm,cy_mm,cz_mm"};int i=0;
  foreach(ComponentOccurrence o in d.ComponentDefinition.Occurrences){var mp=o.MassProperties;var c=mp.CenterOfMass;rows.Add(i+","+o.Name+","+mp.Mass+","+c.X*10+","+c.Y*10+","+c.Z*10);i++;}
  File.WriteAllLines(P("01_Calculos","MASAS_CAD_"+pose+".csv"),rows);
  var m=d.ComponentDefinition.MassProperties;var cm=m.CenterOfMass;Log("TOTAL mass="+m.Mass+" CGmm="+cm.X*10+","+cm.Y*10+","+cm.Z*10);
 }
}

