const express = require("express");
const path = require("path");

const app = express();
const PORT = 3000;

// Sert tous les fichiers du dossier public
app.use(express.static(path.join(__dirname, "public")));

app.listen(PORT, () => {
  console.log(`Serveur lancé : http://localhost:${PORT}`);
});
