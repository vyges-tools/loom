//! LEF58 same-mask spacing rules, read from a layer's raw `PROPERTY LEF58_*` strings.
//!
//! A multi-patterned layer states how far apart two shapes on the SAME mask must be, and those
//! rules are what a colouring has to honour. odb (pin 4155ac31014) drops four of their forms
//! with a warning, measured on a synthetic tech LEF:
//!
//! | form | odb |
//! |---|---|
//! | `SPACING s SAMEMASK` | ODB-0254 unsupported, dropped |
//! | `SPACINGTABLE TWOWIDTHS SAMEMASK …` | ODB-0256 unsupported, dropped |
//! | `FORBIDDENSPACING min max SAMEMASK …` | ODB-0438 parse mismatch, dropped |
//! | `EOLEXTENSIONSPACING s SAMEMASK …` | ODB-0260 parse mismatch, dropped |
//! | `SPACING s ENDOFLINE w … SAMEMASK …` | kept (end-of-line rule, same-mask flag) |
//!
//! This module reads the four dropped forms and nothing else; the end-of-line form and every
//! rule without `SAMEMASK` are odb's. Values stay in microns, as written. A `SAMEMASK` statement
//! of one of these kinds that does not match its form is kept verbatim in
//! [`SameMaskRules::unread`], never skipped silently.
//!
//! Pure std, unit-tested on synthetic strings.
#![deny(clippy::unwrap_used)]
#![cfg_attr(test, allow(clippy::unwrap_used))]

/// The same-mask rules of one layer, in the order the LEF states them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SameMaskRules {
    /// `SPACING s SAMEMASK ;`: minimum spacing between two same-mask shapes.
    pub spacing: Vec<f64>,
    /// `SPACINGTABLE TWOWIDTHS SAMEMASK …`: spacing by the two widths (and parallel run).
    pub two_widths: Vec<TwoWidthsTable>,
    /// `FORBIDDENSPACING min max SAMEMASK …`: a range of spacings two same-mask shapes may not have.
    pub forbidden: Vec<ForbiddenSpacing>,
    /// `EOLEXTENSIONSPACING s SAMEMASK …`: end-of-line spacing with the line end extended.
    pub eol_extension: Vec<EolExtensionSpacing>,
    /// `SAMEMASK` statements of the four kinds above that did not match their form, verbatim.
    pub unread: Vec<String>,
}

/// One row of a TWOWIDTHS table: `WIDTH w [PRL p] s1 … sN`.
#[derive(Debug, Clone, PartialEq)]
pub struct TwoWidthsRow {
    pub width: f64,
    /// The row's parallel run length; `None` when the row states none.
    pub prl: Option<f64>,
    /// One spacing per row of the table: entry `j` is the spacing against a shape of row `j`'s width.
    pub spacings: Vec<f64>,
}

/// A square TWOWIDTHS table (as many spacings per row as there are rows).
#[derive(Debug, Clone, PartialEq)]
pub struct TwoWidthsTable {
    pub rows: Vec<TwoWidthsRow>,
}

impl TwoWidthsTable {
    /// The table's first entry: the spacing of two minimum-width shapes. The reference's own
    /// TWOWIDTHS rule (drt `frSpacingTableTwConstraint::findMin`) reads `front().front()` the
    /// same way. `None` only for an empty table, which the reader never builds.
    #[must_use]
    pub fn min_spacing(&self) -> Option<f64> {
        self.rows.first()?.spacings.first().copied()
    }
}

/// `FORBIDDENSPACING min max SAMEMASK [WIDTH w] [WITHIN d] [PRL p] [TWOEDGES t]`, the optional
/// parts in odb's own grammar order (`lefTechLayerForbiddenSpacingRuleParser`).
#[derive(Debug, Clone, PartialEq)]
pub struct ForbiddenSpacing {
    pub min: f64,
    pub max: f64,
    pub width: Option<f64>,
    pub within: Option<f64>,
    pub prl: Option<f64>,
    pub two_edges: Option<f64>,
}

/// `EOLEXTENSIONSPACING s SAMEMASK [PARALLELONLY] { ENDOFLINE w EXTENSION e … } …`.
#[derive(Debug, Clone, PartialEq)]
pub struct EolExtensionSpacing {
    pub spacing: f64,
    pub parallel_only: bool,
    pub entries: Vec<EolExtensionEntry>,
}

/// One `ENDOFLINE w EXTENSION e… [MINLENGTH l] [TWOSIDES]` entry.
#[derive(Debug, Clone, PartialEq)]
pub struct EolExtensionEntry {
    pub eol_width: f64,
    /// The numbers after `EXTENSION`, as written. odb's grammar (which has no SAMEMASK form)
    /// takes one; LEF files also write a second, which is kept here uninterpreted.
    pub extension: Vec<f64>,
    pub min_length: Option<f64>,
    pub two_sides: bool,
}

impl SameMaskRules {
    /// True when the layer states no same-mask rule this module reads (and none it could not).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.spacing.is_empty()
            && self.two_widths.is_empty()
            && self.forbidden.is_empty()
            && self.eol_extension.is_empty()
            && self.unread.is_empty()
    }

    /// The minimum spacing two minimum-width same-mask shapes must keep: the largest of the
    /// plain `SPACING s SAMEMASK` values and each TWOWIDTHS table's first entry. `None` when the
    /// layer states neither. The checker uses the full tables; this is the single number a
    /// colouring pass screens with.
    #[must_use]
    pub fn min_spacing(&self) -> Option<f64> {
        let tw = self.two_widths.iter().filter_map(TwoWidthsTable::min_spacing);
        self.spacing.iter().copied().chain(tw).reduce(f64::max)
    }

    /// Read one `PROPERTY <name> "<value>"` of a layer. `value` is the string without its quotes;
    /// it may hold several `;`-terminated statements. Properties other than the four kinds and
    /// statements without `SAMEMASK` are ignored.
    pub fn read_property(&mut self, name: &str, value: &str) {
        let name = name.to_ascii_uppercase();
        for stmt in value.split(';') {
            let toks: Vec<String> = stmt.split_whitespace().map(str::to_ascii_uppercase).collect();
            if !toks.iter().any(|t| t == "SAMEMASK") {
                continue;
            }
            let t: Vec<&str> = toks.iter().map(String::as_str).collect();
            let read = match (name.as_str(), t.first().copied()) {
                ("LEF58_SPACING", Some("SPACING")) => {
                    // the end-of-line form is odb's (it keeps its SAMEMASK flag)
                    if t.contains(&"ENDOFLINE") {
                        continue;
                    }
                    read_spacing(&t).map(|s| self.spacing.push(s))
                }
                ("LEF58_SPACINGTABLE", Some("SPACINGTABLE")) => {
                    // PARALLELRUNLENGTH … SAMEMASK is odb's (dbTechLayerSpacingTablePrlRule)
                    if t.get(1) == Some(&"PARALLELRUNLENGTH") {
                        continue;
                    }
                    read_two_widths(&t).map(|tw| self.two_widths.push(tw))
                }
                ("LEF58_FORBIDDENSPACING", Some("FORBIDDENSPACING")) => {
                    read_forbidden(&t).map(|f| self.forbidden.push(f))
                }
                ("LEF58_EOLEXTENSIONSPACING", Some("EOLEXTENSIONSPACING")) => {
                    read_eol_extension(&t).map(|e| self.eol_extension.push(e))
                }
                _ => continue,
            };
            if read.is_none() {
                self.unread.push(stmt.split_whitespace().collect::<Vec<_>>().join(" "));
            }
        }
    }
}

fn num(t: Option<&&str>) -> Option<f64> {
    t.and_then(|s| s.parse::<f64>().ok())
}

/// `SPACING s SAMEMASK`, exactly.
fn read_spacing(t: &[&str]) -> Option<f64> {
    match t {
        ["SPACING", s, "SAMEMASK"] => s.parse().ok(),
        _ => None,
    }
}

/// `SPACINGTABLE TWOWIDTHS SAMEMASK { WIDTH w [PRL p] s1 … sN } …`, square.
fn read_two_widths(t: &[&str]) -> Option<TwoWidthsTable> {
    let ["SPACINGTABLE", "TWOWIDTHS", "SAMEMASK", rest @ ..] = t else { return None };
    let mut rows = Vec::new();
    let mut i = 0;
    while i < rest.len() {
        if rest[i] != "WIDTH" {
            return None;
        }
        let width = num(rest.get(i + 1))?;
        i += 2;
        let mut prl = None;
        if rest.get(i) == Some(&"PRL") {
            prl = Some(num(rest.get(i + 1))?);
            i += 2;
        }
        let mut spacings = Vec::new();
        while let Some(v) = num(rest.get(i)) {
            spacings.push(v);
            i += 1;
        }
        rows.push(TwoWidthsRow { width, prl, spacings });
    }
    let n = rows.len();
    (n > 0 && rows.iter().all(|r| r.spacings.len() == n)).then_some(TwoWidthsTable { rows })
}

/// `FORBIDDENSPACING min max SAMEMASK [WIDTH w] [WITHIN d] [PRL p] [TWOEDGES t]`.
fn read_forbidden(t: &[&str]) -> Option<ForbiddenSpacing> {
    let ["FORBIDDENSPACING", lo, hi, "SAMEMASK", rest @ ..] = t else { return None };
    let mut f = ForbiddenSpacing {
        min: lo.parse().ok()?,
        max: hi.parse().ok()?,
        width: None,
        within: None,
        prl: None,
        two_edges: None,
    };
    let mut i = 0;
    for (kw, slot) in [
        ("WIDTH", &mut f.width),
        ("WITHIN", &mut f.within),
        ("PRL", &mut f.prl),
        ("TWOEDGES", &mut f.two_edges),
    ] {
        if rest.get(i) == Some(&kw) {
            *slot = Some(num(rest.get(i + 1))?);
            i += 2;
        }
    }
    (i == rest.len()).then_some(f)
}

/// `EOLEXTENSIONSPACING s SAMEMASK [PARALLELONLY] { ENDOFLINE w EXTENSION e… [MINLENGTH l] [TWOSIDES] } …`.
fn read_eol_extension(t: &[&str]) -> Option<EolExtensionSpacing> {
    let ["EOLEXTENSIONSPACING", s, "SAMEMASK", rest @ ..] = t else { return None };
    let mut e = EolExtensionSpacing { spacing: s.parse().ok()?, parallel_only: false, entries: Vec::new() };
    let mut i = 0;
    if rest.first() == Some(&"PARALLELONLY") {
        e.parallel_only = true;
        i = 1;
    }
    while i < rest.len() {
        if rest[i] != "ENDOFLINE" || rest.get(i + 2) != Some(&"EXTENSION") {
            return None;
        }
        let eol_width = num(rest.get(i + 1))?;
        i += 3;
        let mut extension = Vec::new();
        while let Some(v) = num(rest.get(i)) {
            extension.push(v);
            i += 1;
        }
        if extension.is_empty() {
            return None;
        }
        let mut min_length = None;
        if rest.get(i) == Some(&"MINLENGTH") {
            min_length = Some(num(rest.get(i + 1))?);
            i += 2;
        }
        let two_sides = rest.get(i) == Some(&"TWOSIDES");
        if two_sides {
            i += 1;
        }
        e.entries.push(EolExtensionEntry { eol_width, extension, min_length, two_sides });
    }
    (!e.entries.is_empty()).then_some(e)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(name: &str, value: &str) -> SameMaskRules {
        let mut r = SameMaskRules::default();
        r.read_property(name, value);
        r
    }

    #[test]
    fn plain_same_mask_spacing() {
        let r = read("LEF58_SPACING", "SPACING 0.12 SAMEMASK ;");
        assert_eq!(r.spacing, vec![0.12]);
        assert_eq!(r.min_spacing(), Some(0.12));
        assert!(r.unread.is_empty());
    }

    /// The end-of-line SAMEMASK form and rules without SAMEMASK are odb's: not read, not unread.
    #[test]
    fn forms_odb_keeps_are_left_to_odb() {
        let r = read(
            "LEF58_SPACING",
            "SPACING 0.07 ENDOFLINE 0.06 WITHIN 0.02 SAMEMASK ENDTOEND 0.08 ; SPACING 0.05 ;",
        );
        assert!(r.is_empty(), "{r:?}");
        let r = read("LEF58_SPACINGTABLE", "SPACINGTABLE PARALLELRUNLENGTH SAMEMASK 0 WIDTH 0 0.1 ;");
        assert!(r.is_empty(), "{r:?}");
    }

    /// Rows `WIDTH w [PRL p] s…`; entry `j` of a row pairs it with row `j`'s width. The first
    /// entry is the minimum-width pair's spacing.
    #[test]
    fn two_widths_same_mask_table() {
        let r = read(
            "LEF58_SPACINGTABLE",
            "SPACINGTABLE TWOWIDTHS SAMEMASK WIDTH 0.0 0.11 0.13 WIDTH 0.08 PRL 0.2 0.13 0.15 ;",
        );
        assert_eq!(
            r.two_widths,
            vec![TwoWidthsTable {
                rows: vec![
                    TwoWidthsRow { width: 0.0, prl: None, spacings: vec![0.11, 0.13] },
                    TwoWidthsRow { width: 0.08, prl: Some(0.2), spacings: vec![0.13, 0.15] },
                ]
            }]
        );
        assert_eq!(r.min_spacing(), Some(0.11));
    }

    #[test]
    fn a_ragged_two_widths_table_is_unread_not_guessed() {
        let r = read("LEF58_SPACINGTABLE", "SPACINGTABLE TWOWIDTHS SAMEMASK WIDTH 0 0.1 WIDTH 0.1 0.2 0.3 ;");
        assert!(r.two_widths.is_empty());
        assert_eq!(r.unread, vec!["SPACINGTABLE TWOWIDTHS SAMEMASK WIDTH 0 0.1 WIDTH 0.1 0.2 0.3".to_string()]);
    }

    #[test]
    fn forbidden_same_mask_spacing() {
        let r = read("LEF58_FORBIDDENSPACING", "FORBIDDENSPACING 0.2 0.25 SAMEMASK WIDTH 0.06 PRL 0.3 TWOEDGES 0.4 ;");
        assert_eq!(
            r.forbidden,
            vec![ForbiddenSpacing {
                min: 0.2,
                max: 0.25,
                width: Some(0.06),
                within: None,
                prl: Some(0.3),
                two_edges: Some(0.4)
            }]
        );
        // out of odb's keyword order: unread
        let r = read("LEF58_FORBIDDENSPACING", "FORBIDDENSPACING 0.2 0.25 SAMEMASK PRL 0.3 WIDTH 0.06 ;");
        assert!(r.forbidden.is_empty());
        assert_eq!(r.unread.len(), 1);
    }

    #[test]
    fn eol_extension_same_mask_spacing() {
        let r = read(
            "LEF58_EOLEXTENSIONSPACING",
            "EOLEXTENSIONSPACING 0.09 SAMEMASK PARALLELONLY ENDOFLINE 0.06 EXTENSION 0.03 0.04 MINLENGTH 0.1 TWOSIDES ;",
        );
        assert_eq!(
            r.eol_extension,
            vec![EolExtensionSpacing {
                spacing: 0.09,
                parallel_only: true,
                entries: vec![EolExtensionEntry {
                    eol_width: 0.06,
                    extension: vec![0.03, 0.04],
                    min_length: Some(0.1),
                    two_sides: true
                }]
            }]
        );
        // these rules do not set the screening spacing
        assert_eq!(r.min_spacing(), None);
    }

    #[test]
    fn several_statements_in_one_property_and_the_largest_minimum_wins() {
        let r = read(
            "lef58_spacing",
            "SPACING 0.12 SAMEMASK ;\n SPACING 0.14 SAMEMASK ;\n SPACING 0.2 SAMEMASK PARALLELOVERLAP ;",
        );
        assert_eq!(r.spacing, vec![0.12, 0.14]);
        assert_eq!(r.min_spacing(), Some(0.14));
        assert_eq!(r.unread, vec!["SPACING 0.2 SAMEMASK PARALLELOVERLAP".to_string()]);
    }
}
