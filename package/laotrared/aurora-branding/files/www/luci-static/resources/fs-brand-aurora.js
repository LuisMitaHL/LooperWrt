'use strict';
'require baseclass';

/*
 * fs-brand-aurora — LaOtraRed Aurora brand mark for the footstrap theme.
 *
 * Companion plugin, registered via `uci add_list
 * footstrap.settings.plugin='fs-brand-aurora'` (see 50_aurora-branding).
 * Replaces the inline OpenWrt mark in .fs-logo with the Aurora glyph,
 * picking the dark/light variant from :root[data-darkmode] — the same
 * attribute the theme's own CSS reads, so auto, manual and per-browser
 * overrides are all followed. A MutationObserver re-applies across
 * SPA re-renders and mode flips.
 *
 * Seams used (all stable/documented): .fs-logo (styles/theme/10-chrome.css),
 * data-darkmode (partials/head.ut), the SVG favicon link for the asset base.
 * No theme files are modified.
 *
 * Licensed under the Apache License, Version 2.0.
 */
return baseclass.extend({
	__init__: function() {
		this.base = this.mediaBase();
		this.paint();
		this.observe();
	},

	/* Asset base derived from the theme's own SVG favicon link, so this
	 * keeps working if the theme is ever served from another prefix. */
	mediaBase: function() {
		var l = document.querySelector('link[rel="icon"][sizes="any"]');
		if (l && l.href) {
			var href = String(l.href).split('?')[0];
			if (href.slice(-8) === 'logo.svg')
				return href.slice(0, -8);
		}
		return '/luci-static/footstrap/';
	},

	want: function() {
		var dark = document.documentElement.getAttribute('data-darkmode') === 'true';
		return this.base + (dark ? 'img/aurora-dark.svg' : 'img/aurora-light.svg');
	},

	paintBox: function(box) {
		var img = box.querySelector('img[data-aurora]');
		if (!img) {
			while (box.firstChild)
				box.removeChild(box.firstChild);
			img = document.createElement('img');
			img.setAttribute('data-aurora', '1');
			img.setAttribute('alt', '');
			img.setAttribute('width', '20');
			img.setAttribute('height', '20');
			img.style.width = '20px';
			img.style.height = '20px';
			box.appendChild(img);
		}
		var src = this.want();
		if (img.getAttribute('src') !== src)
			img.setAttribute('src', src);
	},

	paint: function() {
		var boxes = document.querySelectorAll('.fs-logo');
		for (var i = 0; i < boxes.length; i++)
			this.paintBox(boxes[i]);
	},

	observe: function() {
		var self = this;
		var obs = new MutationObserver(function(muts) {
			for (var i = 0; i < muts.length; i++) {
				var m = muts[i];
				if (m.type === 'attributes') {
					self.paint();
					break;
				}
				var added = m.addedNodes || [];
				for (var j = 0; j < added.length; j++) {
					var n = added[j];
					if (n.nodeType !== 1)
						continue;
					if ((typeof n.matches === 'function' && n.matches('.fs-logo')) ||
					    (typeof n.querySelector === 'function' && n.querySelector('.fs-logo'))) {
						self.paint();
						break;
					}
				}
			}
		});
		obs.observe(document.documentElement, { attributes: true, attributeFilter: ['data-darkmode'] });
		if (document.body) {
			obs.observe(document.body, { childList: true, subtree: true });
		}
		else {
			document.addEventListener('DOMContentLoaded', function() {
				obs.observe(document.body, { childList: true, subtree: true });
				self.paint();
			});
		}
	}
});
