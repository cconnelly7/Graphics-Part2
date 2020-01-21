
class pts // class for manipulaitng and displaying pointclouds or polyloops in 3D 
  { 
    int maxnv = 16000;                 //  max number of vertices
    pt[] G = new pt [maxnv];           // geometry table (vertices)
    char[] L = new char [maxnv];             // labels of points
    vec [] LL = new vec[ maxnv];  // displacement vectors
    Boolean loop=true;          // used to indicate closed loop 3D control polygons
    int pv =0,     // picked vertex index,
        iv=0,      //  insertion vertex index
        dv = 0,   // dancer support foot index
        nv = 0,    // number of vertices currently used in P
        pp=1; // index of picked vertex

  pts() {}
  pts declare() 
    {
    for (int i=0; i<maxnv; i++) G[i]=P(); 
    for (int i=0; i<maxnv; i++) LL[i]=V(); 
    return this;
    }     // init all point objects
  pts empty() {nv=0; pv=0; return this;}                                 // resets P so that we can start adding points
  pts addPt(pt P, char c) { G[nv].setTo(P); pv=nv; L[nv]=c; nv++;  return this;}          // appends a new point at the end
  pts addPt(pt P) { G[nv].setTo(P); pv=nv; L[nv]='f'; nv++;  return this;}          // appends a new point at the end
  pts addPt(float x,float y) { G[nv].x=x; G[nv].y=y; pv=nv; nv++; return this;} // same byt from coordinates
  pts copyFrom(pts Q) {empty(); nv=Q.nv; for (int v=0; v<nv; v++) G[v]=P(Q.G[v]); return this;} // set THIS as a clone of Q

  pts resetOnCircle(int k, float r)  // sets THIS to a polyloop with k points on a circle of radius r around origin
    {
    empty(); // resert P
    pt C = P(); // center of circle
    for (int i=0; i<k; i++) addPt(R(P(C,V(0,-r,0)),2.*PI*i/k,C)); // points on z=0 plane
    pv=0; // picked vertex ID is set to 0
    return this;
    } 
  // ********* PICK AND PROJECTIONS *******  
  int SETppToIDofVertexWithClosestScreenProjectionTo(pt M)  // sets pp to the index of the vertex that projects closest to the mouse 
    {
    pp=0; 
    for (int i=1; i<nv; i++) if (d(M,ToScreen(G[i]))<=d(M,ToScreen(G[pp]))) pp=i; 
    return pp;
    }
  pts showPicked() {show(G[pv],23); return this;}
  pt closestProjectionOf(pt M)    // Returns 3D point that is the closest to the projection but also CHANGES iv !!!!
    {
    pt C = P(G[0]); float d=d(M,C);       
    for (int i=1; i<nv; i++) if (d(M,G[i])<=d) {iv=i; C=P(G[i]); d=d(M,C); }  
    for (int i=nv-1, j=0; j<nv; i=j++) { 
       pt A = G[i], B = G[j];
       if(projectsBetween(M,A,B) && disToLine(M,A,B)<d) {d=disToLine(M,A,B); iv=i; C=projectionOnLine(M,A,B);}
       } 
    return C;    
    }

  // ********* MOVE, INSERT, DELETE *******  
  pts insertPt(pt P) { // inserts new vertex after vertex with ID iv
    for(int v=nv-1; v>iv; v--) {G[v+1].setTo(G[v]);  L[v+1]=L[v];}
     iv++; 
     G[iv].setTo(P);
     L[iv]='f';
     nv++; // increments vertex count
     return this;
     }
  pts insertClosestProjection(pt M) {  
    pt P = closestProjectionOf(M); // also sets iv
    insertPt(P);
    return this;
    }
  pts deletePicked() 
    {
    for(int i=pv; i<nv; i++) 
      {
      G[i].setTo(G[i+1]); 
      L[i]=L[i+1]; 
      }
    pv=max(0,pv-1); 
    nv--;  
    return this;
    }
  pts setPt(pt P, int i) { G[i].setTo(P); return this;}
  
  pts drawBalls(float r) {for (int v=0; v<nv; v++) show(G[v],r); return this;}
  pts showPicked(float r) {show(G[pv],r); return this;}
  pts drawClosedCurve(float r) 
    {
    fill(dgreen);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    fill(magenta);
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    pushMatrix(); //translate(0,0,1); 
    scale(1,1,0.03);  
    fill(grey);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    popMatrix();
    return this;
    }
  pts set_pv_to_pp() {pv=pp; return this;}
  pts movePicked(vec V) { G[pv].add(V); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts setPickedTo(pt Q) { G[pv].setTo(Q); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts moveAll(vec V) {for (int i=0; i<nv; i++) G[i].add(V); return this;};   
  pt Picked() {return G[pv];} 
  pt Pt(int i) {if(0<=i && i<nv) return G[i]; else return G[0];} 

  // ********* I/O FILE *******  
 void savePts(String fn) 
    {
    String [] inppts = new String [nv+1];
    int s=0;
    inppts[s++]=str(nv);
    for (int i=0; i<nv; i++) {inppts[s++]=str(G[i].x)+","+str(G[i].y)+","+str(G[i].z)+","+L[i];}
    saveStrings(fn,inppts);
    };
  
  void loadPts(String fn) 
    {
    println("loading: "+fn); 
    String [] ss = loadStrings(fn);
    String subpts;
    int s=0;   int comma, comma1, comma2;   float x, y;   int a, b, c;
    nv = int(ss[s++]); print("nv="+nv);
    for(int k=0; k<nv; k++) 
      {
      int i=k+s; 
      //float [] xy = float(split(ss[i],",")); 
      String [] SS = split(ss[i],","); 
      G[k].setTo(float(SS[0]),float(SS[1]),float(SS[2]));
      L[k]=SS[3].charAt(0);
      }
    pv=0;
    };
 
  // Dancer
  void setPicekdLabel(char c) {L[pp]=c;}
  


  void setFifo() 
    {
    _LookAtPt.reset(G[dv],60);
    }              


  void next() {dv=n(dv);}
  int n(int v) {return (v+1)%nv;}
  int p(int v) {if(v==0) return nv-1; else return v-1;}
  int addMod(int v) {return (v+1)%nv;}
  int mod(int v) {
    int res = (v-1)%nv;
    if (res < 0) {
      res = res + nv;
    }
    return res;
  }
  
  pts subdivideDemoInto(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
      Q.addPt(P(G[i])); 
      Q.addPt(P(G[i],G[n(i)])); 
      //...
      }
    return this;
    }  
  pts subdivideDemoIntoTest(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
      Q.addPt(B(G[p(i)], G[i], G[n(i)], 1)); 
      Q.addPt(F(G[p(i)], G[i], G[n(i)], G[n(n(i))], 1)); 
      //...
      }
    return this;
    } 
  
  void displaySkater() 
      {
      if(showCurve) {fill(yellow); for (int j=0; j<nv; j++) caplet(G[j],6,G[n(j)],6); }
      pt[] B = new pt [nv];           // geometry table (vertices)
      for (int j=0; j<nv; j++) B[j]=P(G[j],V(0,0,100));
      if(showPath) {fill(lime); for (int j=0; j<nv; j++) caplet(B[j],6,B[n(j)],6);} 
      if(showKeys) {fill(cyan); for (int j=0; j<nv; j+=4) arrow(B[j],G[j],3);}
      
      if(animating) {
        f=n(f);
      }
      
      if (walkingF == 0) {
        walkingStartPoint = f;
      }  
      
      fill(cyan); for (int j=0; j<nv; j++) caplet(this.floorCenter(j),6,this.floorCenter(n(j)),6);

      if(showSkater) 
        {
        pt floorCenter = floorCenter(f);
        pt nextFloorCenter = floorCenter(n(f));
        
        vec forward = new vec(G[n(f)].x - G[f].x, G[n(f)].y - G[f].y, G[n(f)].z - G[f].z);
        vec forwardNorm = U(V(forward));
        
        vec bodyAngle = U(new vec(G[f].x - floorCenter.x, G[f].y - floorCenter.y, G[f].z - floorCenter.z));
        pt torsoTop = new pt(G[f].x, G[f].y, G[f].z).add(V(bodyAngle).mul(150));
        pt head = P(torsoTop).add(V(bodyAngle).mul(60));
        fill(red); caplet(G[f], 40, torsoTop , 50);
        fill(red); sphere(torsoTop, 50);
        fill(red); sphere(head, 45);
        fill(red); sphere(G[f], 15);
        fill(red); sphere(floorCenter, 8);
        //for (int v = 0; v < nv; v+=16) {
        //  fill(brown); sphere(floorCenter(v), 15);
        //}
        vec orthogonalVec = new vec(-(nextFloorCenter.y - floorCenter.y), nextFloorCenter.x - floorCenter.x, 0);
        
        //LEGS
        vec shoulderAngle = cross(forwardNorm, bodyAngle);
        
        pt leftHip = P(G[f]).add(V(shoulderAngle).mul(20));
        pt rightHip = P(G[f]).sub(V(shoulderAngle).mul(20));
        
        pt leftFootOnLine = leftFoot(walkingStartPoint, walkingF, leftHip);
        pt rightFootOnLine = rightFoot(walkingStartPoint, walkingF, rightHip);
        
        pt leftFootFinal = leftFootOnLine.sub(orthogonalVec);
        pt rightFootFinal = rightFootOnLine.add(orthogonalVec);
        
        fill(brown); sphere(leftFootFinal, 20);
        fill(brown); sphere(rightFootFinal, 20);
        
        fill(green); sphere(leftHip, 40);
        fill(blue); sphere(rightHip, 40);

       
        int leftKneeMul;
        int rightKneeMul;
        if (!straightLegs)
        {
          leftKneeMul = leftKneeMultiplier(walkingF);
          rightKneeMul = rightKneeMultiplier(walkingF);
        }
        else {
          leftKneeMul = 0;
          rightKneeMul = 0;
        }
        
        
        pt leftKnee = P(P(leftFootFinal, V(forward).mul(leftKneeMul)),leftHip);
        pt rightKnee = P(P(rightFootFinal, V(forward).mul(rightKneeMul)),rightHip);
        
        fill(green); caplet(leftFootFinal, 20, leftKnee, 30);
        fill(blue); caplet(rightFootFinal, 20, rightKnee, 30);
        fill(green); caplet(leftKnee, 30, leftHip, 40);
        fill(blue); caplet(rightKnee, 30, rightHip, 40);
        
        //ARMS
        vec leftArmVec = U(V(G[f], rightFootFinal));
        vec rightArmVec = U(V(G[f], leftFootFinal));
        
        vec bentLeftAngle = U(V(G[f], P(rightFootFinal).add(V(forward).mul(5))));
        vec bentRightAngle = U(V(G[f], P(leftFootFinal).add(V(forward).mul(5))));
        
        pt leftShoulder = P(torsoTop).add(V(shoulderAngle).mul(55));
        pt rightShoulder = P(torsoTop).sub(V(shoulderAngle).mul(55));
        
        pt leftElbow = P(leftShoulder).add(V(leftArmVec).mul(100));
        pt rightElbow = P(rightShoulder).add(V(rightArmVec).mul(100));
        
        pt leftHand = P(leftElbow).add(V(bentLeftAngle).mul(100));
        pt rightHand = P(rightElbow).add(V(bentRightAngle).mul(100));
        
        fill (red); caplet(leftShoulder, 10, leftElbow, 10);
        fill (red); caplet(rightShoulder, 10, rightElbow, 10);
        fill (red); caplet(leftElbow, 10, leftHand, 10);
        fill (red); caplet(rightElbow, 10, rightHand, 10);
        }
      
      
      else {
      
        fill(red); arrow(this.floorCenter(f), G[f],20);
      }
      for(int v = 0; v < nv; v++)
      {
       
        fill(green); arrow(this.floorCenter(v), G[v],5);
      }
      
      walkingF += 1;
        if (walkingF == 18) {
          walkingF = 0;
        }
   }
   pt leftFoot(int walkingStartPoint, int walkingF, pt leftHip) {
     if (walkingF < 12) {
       return floorCenter(mod(walkingStartPoint + 7));
     } else {
       pt footOnLine = floorCenter(mod(walkingStartPoint + 7  + (walkingF -12) *3));
       vec legVec = V(footOnLine, leftHip);
       
       float raisedFootMult;
       if (walkingF == 12) { raisedFootMult = .1; }
       if (walkingF == 13) { raisedFootMult = .15; }
       if (walkingF == 14) { raisedFootMult = .2; }
       if (walkingF == 15) { raisedFootMult = .2; }
       if (walkingF == 16) { raisedFootMult = .15; }
       else { raisedFootMult = .1; }
       
       footOnLine.add(legVec.mul(raisedFootMult));
       
       return footOnLine;
     }
   }
   pt rightFoot(int walkingStartPoint, int walkingF, pt rightHip) {
     if (walkingF < 3) {
       return floorCenter(mod(walkingStartPoint - 1));
     } else if (walkingF < 9) {
       pt footOnLine= floorCenter(mod(walkingStartPoint - 1 + (walkingF -3) * 3));
        vec legVec = V(footOnLine, rightHip);
        
        float raisedFootMult;
       if (walkingF == 3) { raisedFootMult = .1; }
       if (walkingF == 4) { raisedFootMult = .15; }
       if (walkingF == 5) { raisedFootMult = .2; }
       if (walkingF == 6) { raisedFootMult = .2; }
       if (walkingF == 7) { raisedFootMult = .15; }
       else { raisedFootMult = .1; }
       footOnLine.add(legVec.mul(raisedFootMult));
        return footOnLine;
     }
      else {
        return floorCenter(mod(walkingStartPoint + 15 ));
        
     }
     
   }
   int leftKneeMultiplier(int walkingF) {
     if (walkingF == 12) {
       return 2;
     }
     else if (walkingF == 13) {
       return 3;
     }
     else if (walkingF == 14) {
       return 4;
     }
     else if (walkingF == 15) {
       return 4;
     }
     else if (walkingF == 16) {
       return 3;
     }
     else if (walkingF == 17) {
       return 2;
     }
     else {
       return 1;
     } 
   }
   
   int rightKneeMultiplier(int walkingF) {
     if (walkingF == 3) {
       return 2;
     }
     else if (walkingF == 4) {
       return 3;
     }
     else if (walkingF == 5) {
       return 4;
     }
     else if (walkingF == 6) {
       return 4;
     }
     else if (walkingF == 7) {
       return 3;
     }
     else if (walkingF == 8) {
       return 2;
     }
     else {
       return 1;
     } 
   }
   
   pt floorCenter(int v) {
        pt Apt = G[p(v)];
        pt Bpt = G[v];
        pt Cpt = G[n(v)];
        vec BA = new vec(Apt.x - Bpt.x, Apt.y - Bpt.y, Apt.z - Bpt.z);
        vec BC = new vec(Cpt.x - Bpt.x, Cpt.y - Bpt.y, Cpt.z - Bpt.z);
        
        vec Acceleration = (BA.add(BC)).div(2.0).mul(50);
        
        return new pt(G[v].x - Acceleration.x, G[v].y - Acceleration.y, 0);
   }

        

} // end of pts class
