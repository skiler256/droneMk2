// GCS/codes-table.js
// ─────────────────────────────────────────────────────────────────────────────
// Charge INDEX.csv côté serveur (même source de vérité que tools/gen_codes.py
// et GCS/public/js/link.js côté navigateur) — sert uniquement à calculer
// quels voyants ESP32 allumer pour les codes actifs (colonne Panel), cf.
// GCS/server.js. Pas de dépendance ajoutée : parsing CSV minimal, le format
// est simple (';' délimité, jamais de champ contenant ';').

const fs = require("fs");

const COMPONENTS = { FC: 1, SF: 2, NV: 3, MC: 4, SM: 5, MV: 6, GW: 7 };
const CATEGORIES = { P: 1, S: 2, A: 3, C: 4, O: 5 };

/**
 * @param {string} csvPath chemin vers INDEX.csv
 * @returns {Map<number, {code:string, name:string, severity:string, panel:string[]}>}
 *          clé = CODES::Code packé (même encodage que le paquet CODE::code).
 */
function loadCodesTable(csvPath) {
  const text = fs.readFileSync(csvPath, "utf-8");
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  const table = new Map();

  for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(";");
    const code = cols[0];
    const name = cols[3];
    const severity = (cols[5] || "").trim();
    const panel = (cols[7] || "").trim();
    const m = code && code.match(/^([A-Z]{2})([A-Z])(\d{3})$/);
    if (!m) continue;

    const packed =
      (((COMPONENTS[m[1]] || 0) << 24) | ((CATEGORIES[m[2]] || 0) << 16) | parseInt(m[3], 10)) >>> 0;

    table.set(packed, {
      code,
      name,
      severity,
      panel: panel ? panel.split(",").map((p) => p.trim()).filter(Boolean) : [],
    });
  }

  return table;
}

/** "FCP002" -> uint32 packé, même encodage que CODES::Code::pack() (C++)
 * et le paquet CODE::code. Lève si le code ne parse pas ("XXP999" invalide
 * n'est pas la responsabilité de ce module -- appelant chargé de valider). */
function packCode(codeStr) {
  const m = codeStr.match(/^([A-Z]{2})([A-Z])(\d{3})$/);
  if (!m) throw new Error(`code '${codeStr}' invalide`);
  const comp = COMPONENTS[m[1]];
  const cat = CATEGORIES[m[2]];
  if (!comp || !cat) throw new Error(`code '${codeStr}': composant/catégorie inconnu`);
  return ((comp << 24) | (cat << 16) | parseInt(m[3], 10)) >>> 0;
}

module.exports = { loadCodesTable, packCode };
