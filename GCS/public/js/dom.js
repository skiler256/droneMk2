/* ============================================================================
   dom.js — couche d'acces au DOM.

   Deux idees seulement :

   1. DIRTY-CHECKING. Ecrire dans le DOM est cher, comparer une chaine ne l'est
      pas. Chaque setter memorise la derniere valeur ecrite sur le noeud et ne
      touche au DOM que si elle a change. A 20 Hz sur ~400 champs, ca fait la
      difference entre un dashboard fluide et un dashboard qui rame.

   2. POOLS PREALLOUES. Les listes (barres, ticks, points lidar, lignes de log)
      sont creees une fois au demarrage a leur taille max, puis mises a jour ou
      masquees. Zero createElement dans la boucle, zero churn GC.
      Meme logique que "no dynamic allocation in RT loops" cote embarque.
   ========================================================================== */

(function (global) {
  'use strict';

  var GCS = global.GCS = global.GCS || {};
  var DOM = GCS.dom = {};

  var bindings = Object.create(null);   // data-bind -> element
  var lists    = Object.create(null);   // data-list -> element conteneur

  /* ── Collecte ──────────────────────────────────────────────────────────── */

  DOM.collect = function collect(root) {
    root = root || document;
    var i, els;

    els = root.querySelectorAll('[data-bind]');
    for (i = 0; i < els.length; i++) bindings[els[i].dataset.bind] = els[i];

    els = root.querySelectorAll('[data-list]');
    for (i = 0; i < els.length; i++) lists[els[i].dataset.list] = els[i];
  };

  DOM.el   = function (name) { return bindings[name] || null; };
  DOM.list = function (name) { return lists[name] || null; };

  /* ── Setters a dirty-checking ──────────────────────────────────────────── */

  /* Sur un noeud precis (utilise par les pools, qui n'ont pas de data-bind). */
  DOM.nodeText = function nodeText(el, value) {
    if (!el) return;
    if (el.__txt !== value) { el.__txt = value; el.textContent = value; }
  };

  DOM.nodeStyle = function nodeStyle(el, prop, value) {
    if (!el) return;
    var key = '__st_' + prop;
    if (el[key] !== value) { el[key] = value; el.style.setProperty(prop, value); }
  };

  DOM.nodeAttr = function nodeAttr(el, name, value) {
    if (!el) return;
    var key = '__at_' + name;
    if (el[key] !== value) { el[key] = value; el.setAttribute(name, value); }
  };

  DOM.nodeClass = function nodeClass(el, cls, on) {
    if (!el) return;
    var key = '__cl_' + cls;
    if (el[key] !== on) { el[key] = on; el.classList.toggle(cls, !!on); }
  };

  DOM.nodeShow = function nodeShow(el, on) {
    DOM.nodeClass(el, 'hidden', !on);
  };

  /* Versions par nom de binding. */
  DOM.text  = function (n, v)    { DOM.nodeText(bindings[n], v); };
  DOM.style = function (n, p, v) { DOM.nodeStyle(bindings[n], p, v); };
  DOM.attr  = function (n, a, v) { DOM.nodeAttr(bindings[n], a, v); };
  DOM.cls   = function (n, c, on){ DOM.nodeClass(bindings[n], c, on); };
  DOM.show  = function (n, on)   { DOM.nodeShow(bindings[n], on); };

  /* ── Pools ─────────────────────────────────────────────────────────────── */

  /**
   * Cree `count` noeuds dans le conteneur data-list="name".
   * @param {string}   name    nom du data-list
   * @param {number}   count   taille max du pool
   * @param {Function} factory (index) => HTMLElement
   * @returns {Array<HTMLElement>} pool, ou [] si le conteneur n'existe pas
   */
  DOM.pool = function pool(name, count, factory) {
    var host = lists[name];
    if (!host) { console.warn('[gcs] data-list introuvable :', name); return []; }
    host.textContent = '';
    var arr = new Array(count);
    for (var i = 0; i < count; i++) {
      var el = factory(i);
      host.appendChild(el);
      arr[i] = el;
    }
    return arr;
  };

  /**
   * Applique `items` sur un pool : met a jour les n premiers, masque le reste.
   * @param {Array} poolArr
   * @param {Array} items
   * @param {Function} update (el, item, index) => void
   */
  DOM.applyPool = function applyPool(poolArr, items, update) {
    var n = items ? items.length : 0;
    for (var i = 0; i < poolArr.length; i++) {
      var el = poolArr[i];
      if (i < n) {
        DOM.nodeShow(el, true);
        update(el, items[i], i);
      } else {
        DOM.nodeShow(el, false);
      }
    }
    if (n > poolArr.length) {
      // Le pool est sous-dimensionne : on tronque plutot que d'allouer.
      // Si ca arrive en pratique, augmenter la taille a l'init.
      console.warn('[gcs] pool sature :', n, '>', poolArr.length);
    }
  };

  /* ── Petit utilitaire de construction ──────────────────────────────────── */

  DOM.make = function make(tag, className, html) {
    var el = document.createElement(tag);
    if (className) el.className = className;
    if (html != null) el.innerHTML = html;
    return el;
  };

})(window);
