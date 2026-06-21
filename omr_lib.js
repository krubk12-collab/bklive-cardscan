/* OMR core: homography (4 จุด) + เทมเพลตจัดวางวงฝนอัตโนมัติ (ไม่ซ้อนกัน) */
(function(root){
  function solve(A, b){
    const n=b.length;
    for(let i=0;i<n;i++){
      let mx=i; for(let r=i+1;r<n;r++) if(Math.abs(A[r][i])>Math.abs(A[mx][i])) mx=r;
      [A[i],A[mx]]=[A[mx],A[i]]; [b[i],b[mx]]=[b[mx],b[i]];
      const piv=A[i][i]; if(Math.abs(piv)<1e-12) return null;
      for(let r=0;r<n;r++){ if(r===i) continue; const f=A[r][i]/piv;
        for(let c=i;c<n;c++) A[r][c]-=f*A[i][c]; b[r]-=f*b[i]; }
    }
    const x=new Array(n); for(let i=0;i<n;i++) x[i]=b[i]/A[i][i]; return x;
  }
  function computeH(src, dst){
    const A=[], b=[];
    for(let i=0;i<4;i++){ const {x,y}=src[i], {x:u,y:v}=dst[i];
      A.push([x,y,1,0,0,0,-u*x,-u*y]); b.push(u);
      A.push([0,0,0,x,y,1,-v*x,-v*y]); b.push(v); }
    return solve(A,b);
  }
  function applyH(h, x, y){ const w=h[6]*x+h[7]*y+1; return { x:(h[0]*x+h[1]*y+h[2])/w, y:(h[3]*x+h[4]*y+h[5])/w }; }

  // เทมเพลตกระดาษ (canonical 1000x1414) — คำนวณคอลัมน์/ขนาด/ระยะ ไม่ให้วงซ้อน
  function template(nQ){
    const W=1000, H=1414, m=150;
    const cols = nQ<=12 ? 1 : (nQ<=24 ? 2 : 3);
    const rowsPerCol = Math.ceil(nQ/cols);
    const top=370, bottomLimit=1205;
    const step = Math.min(94, (bottomLimit-top)/Math.max(1,rowsPerCol-1));
    // แนวนอน: แบ่งพื้นที่ 110..980 เป็น cols กลุ่ม (label + 4 วง)
    const areaL=110, areaR=980, gw=(areaR-areaL)/cols;
    const labelW=66, bubbleAreaW=gw-labelW-16, bstep=bubbleAreaW/4;
    const colBase=[], choiceDX=[];
    for(let c=0;c<cols;c++) colBase.push(areaL + c*gw);
    for(let j=0;j<4;j++) choiceDX.push(labelW + bstep*(j+0.5));
    // รัศมีวง: จำกัดด้วยทั้งแนวตั้ง(step)และแนวนอน(bstep)
    const r = Math.max(13, Math.min(26, step*0.40, bstep*0.42));
    const tpl={ W,H, markerSize:m,
      corners:{ tl:{id:246,cx:90,cy:90}, tr:{id:247,cx:910,cy:90}, br:{id:248,cx:910,cy:1324}, bl:{id:249,cx:90,cy:1324} },
      student:{ cx:500, cy:120, size:140 },
      r, cols, rowsPerCol, colBase, choiceDX, labelW, rows:[], nQ };
    for(let i=0;i<nQ;i++){ const c=Math.floor(i/rowsPerCol), rr=i%rowsPerCol; tpl.rows.push({col:c, y:top+rr*step}); }
    tpl.whiteX = colBase[0] + 12; // จุดอ้างอิงสีขาว (ในพื้นที่ label ที่ว่าง) — ผู้เรียกควรใช้ colBase[row.col]+12 ต่อแถว
    return tpl;
  }
  // พิกัด x ของวงตัวเลือก
  function colX(tpl, col, choiceIdx){ return tpl.colBase[col] + tpl.choiceDX[choiceIdx]; }
  // พิกัด x ของเลขข้อ (ป้าย)
  function labelX(tpl, col){ return tpl.colBase[col] + 6; }
  // จุดอ้างอิงสีขาวต่อแถว (ในพื้นที่ label)
  function whiteXOf(tpl, col){ return tpl.colBase[col] + 12; }

  root.OMR={ computeH, applyH, template, colX, labelX, whiteXOf };
})(typeof module!=='undefined'?module.exports:(this.OMR={}, this));
