/* OMR core: homography (4 จุด) + อ่านค่าวงฝน — ใช้ร่วมกันทั้ง POC และเทสต์ */
(function(root){
  // แก้ระบบสมการเชิงเส้น n×n ด้วย Gaussian elimination
  function solve(A, b){
    const n=b.length;
    for(let i=0;i<n;i++){
      // pivot
      let mx=i; for(let r=i+1;r<n;r++) if(Math.abs(A[r][i])>Math.abs(A[mx][i])) mx=r;
      [A[i],A[mx]]=[A[mx],A[i]]; [b[i],b[mx]]=[b[mx],b[i]];
      const piv=A[i][i]; if(Math.abs(piv)<1e-12) return null;
      for(let r=0;r<n;r++){ if(r===i) continue; const f=A[r][i]/piv;
        for(let c=i;c<n;c++) A[r][c]-=f*A[i][c]; b[r]-=f*b[i]; }
    }
    const x=new Array(n); for(let i=0;i<n;i++) x[i]=b[i]/A[i][i]; return x;
  }
  // homography src(4)->dst(4); คืน [h0..h7] (h8=1)
  function computeH(src, dst){
    const A=[], b=[];
    for(let i=0;i<4;i++){
      const {x,y}=src[i], {x:u,y:v}=dst[i];
      A.push([x,y,1,0,0,0,-u*x,-u*y]); b.push(u);
      A.push([0,0,0,x,y,1,-v*x,-v*y]); b.push(v);
    }
    const h=solve(A,b); if(!h) return null;
    return h;
  }
  function applyH(h, x, y){
    const w=h[6]*x+h[7]*y+1;
    return { x:(h[0]*x+h[1]*y+h[2])/w, y:(h[3]*x+h[4]*y+h[5])/w };
  }
  // เทมเพลตกระดาษ (canonical units) — 4 มุม + เลขที่ + ตารางวงฝน
  function template(nQ){
    const W=1000, H=1414;
    const m=150; // ขนาดมาร์กเกอร์
    const tpl={ W,H, markerSize:m,
      corners:{ tl:{id:246,cx:90,cy:90}, tr:{id:247,cx:910,cy:90}, br:{id:248,cx:910,cy:1324}, bl:{id:249,cx:90,cy:1324} },
      student:{ cx:500, cy:120, size:140 },
      r:26, cols:[420,560,700,840], rows:[], labelX:300, whiteX:230 };
    const top=360, step=(nQ<=10)?92:(nQ<=20?52:(nQ<=40?26:18));
    // จัดคอลัมน์ถ้าข้อเยอะ (>20 แบ่ง 2 คอลัมน์)
    if(nQ<=20){ for(let i=0;i<nQ;i++) tpl.rows.push({col:0,y:top+i*step}); }
    else {
      const perCol=Math.ceil(nQ/2); const step2=(perCol<=20?40:(perCol<=30?28:20));
      for(let i=0;i<nQ;i++){ const c=Math.floor(i/perCol), r=i%perCol; tpl.rows.push({col:c,y:top+r*step2}); }
    }
    tpl.nQ=nQ;
    return tpl;
  }
  // หาพิกัด x ของวงตัวเลือก ตาม column ของข้อ (รองรับ 2 คอลัมน์)
  function colX(tpl, rowCol, choiceIdx){
    const base = rowCol===0 ? tpl.cols : tpl.cols.map(x=>x+ (0)); // คอลัมน์ 2 จะ offset ทาง x
    if(tpl.rows.some(r=>r.col===1)){
      // 2 คอลัมน์: คอลัมน์0 ฝั่งซ้าย, คอลัมน์1 ฝั่งขวา
      const left=[300,380,460,540], right=[620,700,780,860];
      return (rowCol===0?left:right)[choiceIdx];
    }
    return tpl.cols[choiceIdx];
  }
  root.OMR={ computeH, applyH, template, colX };
})(typeof module!=='undefined'?module.exports:(this.OMR={}, this));
