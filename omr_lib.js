/* OMR core: homography (4 จุด) + เทมเพลตจัดวางวงฝนอัตโนมัติ (ไม่ซ้อนกัน)
   v2: มุมกระดาษใช้ "สี่เหลี่ยมทึบดำ" (แบบ ZipGrade) แทน ArUco — เล็กลง สวยขึ้น
       ตรวจจับด้วย CV pipeline (grayscale→adaptiveThreshold→findContours→approxPolyDP)
       เลขที่นักเรียน = ตารางฝนเลข (numGrid) พิมพ์ฝนอัตโนมัติในโหมดรายคน */
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
    const W=1000, H=1414, m=80;   // m = ขนาดสี่เหลี่ยมมุม (เล็กลงจากเดิม 150)
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
      corners:{ tl:{cx:90,cy:90}, tr:{cx:910,cy:90}, br:{cx:910,cy:1324}, bl:{cx:90,cy:1324} },
      // ตารางฝนเลขที่ 2 หลัก (หลักสิบ/หลักหน่วย) วางในแถบว่างเหนือหัวกระดาษ (y145,177) — ไม่ชนคำถาม(370)
      numGrid:{ r:11, x0:250, dx:64, rows:[{y:145},{y:177}] },
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
  // พิกัด x ของวงเลขในตารางฝนเลขที่ (digit 0..9)
  function numX(tpl, digit){ return tpl.numGrid.x0 + digit*tpl.numGrid.dx; }

  /* ตรวจจับสี่เหลี่ยมทึบ 4 มุม แล้วจับคู่ tl/tr/br/bl ตามตำแหน่งสุดขอบ
     img = {width,height,data(RGBA)} (ImageData ก็ได้) ; CV = ไลบรารีของ js-aruco2 ; buf = ที่พักภาพถาวร */
  function detectCorners(img, CV, buf){
    buf.grey  = buf.grey  || new CV.Image();
    buf.thres = buf.thres || new CV.Image();
    buf.bin   = buf.bin   || new CV.Image();
    CV.grayscale(img, buf.grey);
    CV.adaptiveThreshold(buf.grey, buf.thres, 2, 7);
    const contours = CV.findContours(buf.thres, buf.bin);
    const w=img.width, h=img.height;
    const minEdge=w*0.012, maxEdge=w*0.17;
    const cands=[];
    for(let i=0;i<contours.length;i++){
      const c=contours[i];
      if(c.length<8) continue;
      const poly=CV.approxPolyDP(c, c.length*0.05);
      if(poly.length!==4 || !CV.isContourConvex(poly)) continue;
      let mn=Infinity, mx=0;
      for(let k=0;k<4;k++){ const a=poly[k], b=poly[(k+1)%4]; const e=Math.hypot(a.x-b.x,a.y-b.y); if(e<mn)mn=e; if(e>mx)mx=e; }
      if(mn<minEdge || mx>maxEdge) continue;   // ขนาดต้องพอเหมาะ (ตัดกล่องหัวกระดาษที่ใหญ่)
      if(mx/mn>1.6) continue;                    // ต้องเกือบจตุรัส (ตัดกล่องยาว/ขีดเส้น)
      const cx=(poly[0].x+poly[1].x+poly[2].x+poly[3].x)/4;
      const cy=(poly[0].y+poly[1].y+poly[2].y+poly[3].y)/4;
      cands.push({cx,cy,s:(mn+mx)/2});
    }
    const n=cands.length;
    if(n<4) return {corners:null, n};
    const ext=(f,want)=>{ let best=cands[0], bv=f(cands[0]);
      for(let i=1;i<n;i++){ const v=f(cands[i]); if((want==='min'&&v<bv)||(want==='max'&&v>bv)){bv=v;best=cands[i];} } return best; };
    const tl=ext(p=>p.cx+p.cy,'min'), br=ext(p=>p.cx+p.cy,'max');
    const tr=ext(p=>p.cx-p.cy,'max'), bl=ext(p=>p.cx-p.cy,'min');
    const four=[tl,tr,br,bl];
    for(let i=0;i<4;i++) for(let j=i+1;j<4;j++) if(four[i]===four[j]) return {corners:null, n}; // ต้องเป็น 4 จุดต่างกัน
    const ss=four.map(p=>p.s);
    if(Math.max(...ss)/Math.min(...ss) > 2.5) return {corners:null, n};   // มุมจริงขนาดใกล้กัน
    let area=0; for(let i=0;i<4;i++){ const a=four[i], b=four[(i+1)%4]; area+=a.cx*b.cy-b.cx*a.cy; } area=Math.abs(area)/2;
    if(area < w*h*0.10) return {corners:null, n};   // สี่เหลี่ยมต้องกินพื้นที่พอ (จ่อให้เต็มใบ)
    return {corners:{ tl:{x:tl.cx,y:tl.cy}, tr:{x:tr.cx,y:tr.cy}, br:{x:br.cx,y:br.cy}, bl:{x:bl.cx,y:bl.cy} }, n};
  }

  root.OMR={ computeH, applyH, template, colX, labelX, whiteXOf, numX, detectCorners };
})(typeof module!=='undefined'?module.exports:(this.OMR={}, this));
