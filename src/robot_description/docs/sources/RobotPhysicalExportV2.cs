using System;
using System.Collections.Generic;
using System.Globalization;
using Inventor;
using File = System.IO.File;

public partial class RobotCad {
 sealed class PhysicalLinkV2 {
  public int Index;
  public string Name;
  public double Mass;
  public double[] LocalCom, HorizontalCom, Api, Principal;
  public double[,] ToNominal;
 }
 static string PhysF(double x) { return x.ToString("G17", CultureInfo.InvariantCulture); }
 static double PhysDet(double[] t, double sign) {
  double a=t[0],b=t[1],c=t[2],d=sign*t[3],e=sign*t[4],f=sign*t[5];
  return a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d;
 }
 static double[,] PhysTensor(double[] t,double sign,double[,] r) {
  double[,] a={{t[0],sign*t[3],sign*t[4]},{sign*t[3],t[1],sign*t[5]},{sign*t[4],sign*t[5],t[2]}};
  var b=new double[3,3];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)for(int l=0;l<3;l++)b[i,j]+=r[i,k]*a[k,l]*r[j,l];
  return b;
 }
 static void PhysRequireNear(Point p,double x,double y,double z,string label) {
  if(Math.Abs(p.X*10-x)>0.001 || Math.Abs(p.Y*10-y)>0.001 || Math.Abs(p.Z*10-z)>0.001)
   throw new InvalidOperationException("Referencia nominal no coincide: "+label+". No se exportan masas horizontales asumidas.");
 }
 // Read-only physical export. No joint drive, grounding or occurrence placement.
 // The nominal frame is the coordinates of each REFERENCIAS part. Six offsets
 // are checked against the actual construction sketch centres before use.
 static void ExportPhysicalFinalV2() {
  var main=MainV1();
  string stamp=DateTime.UtcNow.ToString("o",CultureInfo.InvariantCulture);
  var links=new List<PhysicalLinkV2>();
  double gx=0,gz=0;
  for(int i=0;i<7;i++) {
   if(i>0) { gx+=offsetsX[i-1]; gz+=offsetsZ[i-1]; }
   var sub=LinkV1(main,i); var refs=Find(sub,"REFERENCIAS");
   var pd=(PartComponentDefinition)refs.Definition;
   if(i>0) PhysRequireNear(pd.Sketches["ENTRADA"].SketchCircles[1].Geometry3d.Center,0,0,0,"L"+i+" ENTRADA");
   if(i<6) PhysRequireNear(pd.Sketches["SALIDA"].SketchCircles[1].Geometry3d.Center,offsetsX[i],0,offsetsZ[i],"L"+i+" SALIDA");
   var inv=refs.Transformation.Copy(); inv.Invert();
   var mp=sub.ComponentDefinition.MassProperties;
   mp.CacheResultsOnCompute=false;
   var local=mp.CenterOfMass.Copy(); local.TransformBy(inv);
   var row=new PhysicalLinkV2 {Index=i,Name=main.ComponentDefinition.Occurrences[i+1].Name,Mass=mp.Mass,
    LocalCom=new[]{local.X*.01,local.Y*.01,local.Z*.01},
    HorizontalCom=new[]{local.X*10+gx,local.Y*10,local.Z*10+gz},ToNominal=new double[3,3]};
   for(int a=0;a<3;a++)for(int b=0;b<3;b++)row.ToNominal[a,b]=inv.Cell[a+1,b+1];
   double xx,yy,zz,xy,yz,xz,p1,p2,p3;
   // Autodesk signature is Ixx,Iyy,Izz,Ixy,Iyz,Ixz (not Ixy,Ixz,Iyz).
   mp.XYZMomentsOfInertia(out xx,out yy,out zz,out xy,out yz,out xz);
   mp.PrincipalMomentsOfInertia(out p1,out p2,out p3);
   row.Api=new[]{xx*1e-4,yy*1e-4,zz*1e-4,xy*1e-4,xz*1e-4,yz*1e-4};
   row.Principal=new[]{p1*1e-4,p2*1e-4,p3*1e-4};
   if(!(row.Mass>0) || !(p1>0 && p2>0 && p3>0)) throw new InvalidOperationException("Propiedades fisicas no positivas en L"+i);
   links.Add(row);
  }
  // The API does not promise the UI's integral-sign convention. Determine the
  // common sign from tensor invariants and independently returned principal
  // moments. A near-diagonal link alone cannot identify the sign; the seven-link
  // aggregate does. Reject an ambiguous or inconsistent result.
  double sameError=0,oppositeError=0;
  foreach(var r in links) {
   double largest=Math.Max(r.Principal[0],Math.Max(r.Principal[1],r.Principal[2]));
   double trace=r.Api[0]+r.Api[1]+r.Api[2], principalTrace=r.Principal[0]+r.Principal[1]+r.Principal[2];
   double traceSquare=r.Api[0]*r.Api[0]+r.Api[1]*r.Api[1]+r.Api[2]*r.Api[2]+2*(r.Api[3]*r.Api[3]+r.Api[4]*r.Api[4]+r.Api[5]*r.Api[5]);
   double principalSquare=r.Principal[0]*r.Principal[0]+r.Principal[1]*r.Principal[1]+r.Principal[2]*r.Principal[2];
   if(Math.Abs(trace-principalTrace)>largest*1e-5 || Math.Abs(traceSquare-principalSquare)>largest*largest*1e-5 || largest>principalTrace-largest+largest*1e-5)
    throw new InvalidOperationException("Tensor API incompatible con momentos principales L"+r.Index);
   double scale=Math.Pow(largest,3);
   double det=r.Principal[0]*r.Principal[1]*r.Principal[2];
   sameError+=Math.Abs(PhysDet(r.Api,1)-det)/scale;
   oppositeError+=Math.Abs(PhysDet(r.Api,-1)-det)/scale;
  }
  double sign=sameError<oppositeError?1:-1;
  double chosen=Math.Min(sameError,oppositeError),rejected=Math.Max(sameError,oppositeError);
  if(chosen>1e-5 || rejected<Math.Max(1e-9,chosen*10))
   throw new InvalidOperationException("Convencion de inercia no resuelta contra momentos principales: same="+PhysF(sameError)+" opposite="+PhysF(oppositeError));
  var masses=new List<string>{"link,name,mass_kg,cx_mm,cy_mm,cz_mm,Izz_com_kgm2,reference_frame,export_id"};
  var inertias=new List<string>{"link,mass_kg,com_x_m,com_y_m,com_z_m,Ixx_kgm2,Iyy_kgm2,Izz_kgm2,Ixy_kgm2,Ixz_kgm2,Iyz_kgm2,reference_frame,inertia_convention,export_id"};
  var raw=new List<string>{"link,api_xx_kgm2,api_yy_kgm2,api_zz_kgm2,api_xy_kgm2,api_xz_kgm2,api_yz_kgm2,principal_1_kgm2,principal_2_kgm2,principal_3_kgm2,tensor_offdiagonal_multiplier,reference_frame,export_id"};
  foreach(var r in links) {
   var t=PhysTensor(r.Api,sign,r.ToNominal);
   masses.Add(r.Index+","+r.Name+","+PhysF(r.Mass)+","+PhysF(r.HorizontalCom[0])+","+PhysF(r.HorizontalCom[1])+","+PhysF(r.HorizontalCom[2])+","+PhysF(t[2,2])+",HORIZONTAL_NOMINAL_V2,"+stamp);
   inertias.Add("L"+r.Index+","+PhysF(r.Mass)+","+PhysF(r.LocalCom[0])+","+PhysF(r.LocalCom[1])+","+PhysF(r.LocalCom[2])+","+PhysF(t[0,0])+","+PhysF(t[1,1])+","+PhysF(t[2,2])+","+PhysF(t[0,1])+","+PhysF(t[0,2])+","+PhysF(t[1,2])+",LOCAL_NOMINAL_L"+r.Index+",negative_integral_tensor,"+stamp);
   raw.Add("L"+r.Index+","+string.Join(",",Array.ConvertAll(r.Api,PhysF))+","+string.Join(",",Array.ConvertAll(r.Principal,PhysF))+","+PhysF(sign)+",CAD_LINK_LOCAL,"+stamp);
  }
  File.WriteAllLines(P("01_Calculos","MASAS_CAD_FINAL_HORIZONTAL.csv"),masses);
  File.WriteAllLines(P("01_Calculos","PROPIEDADES_INERCIALES.csv"),inertias);
  File.WriteAllLines(P("01_Calculos","INERCIAS_API_AUDITORIA_FINAL.csv"),raw);
  Log("PHYSICAL_FINAL_V2 seven links; horizontal derived from local frames; no pose change; inertia sign multiplier="+sign+" principal determinant error="+PhysF(chosen));
 }
}
