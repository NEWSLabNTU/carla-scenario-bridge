# Measured terramechanics data to calibrate the soil layer against

Survey for [off-road-surfaces.md](off-road-surfaces.md), 2026-10-24: published, real-world
measurements that the soil layer's classes (`src/isaac_world/config/surface_classes.yaml`)
could be calibrated against. The earlier survey
([survey-localized-deformable-terrain.md](survey-localized-deformable-terrain.md)) chose the
model; this one looks for ground truth. Nothing here was measured by us, and nothing was
fitted yet.

## Purpose

The soil layer runs a Bekker-Wong-Janosi model (pressure-sinkage k_c, k_phi, n; Mohr-Coulomb
cohesion and friction angle; Janosi shear modulus K; Lyasko slip sinkage) under a Tesla
Model 3: a pneumatic road tire around 235/45R18, about 4.5 kN per wheel, inflated near
290 kPa. Today its four soil classes take their parameters from Wong's textbook table and are
calibrated only against two simulators (047 S5, "Calibration against Chrono" in
off-road-surfaces.md). Chrono's SCM and CRM agree on sinkage but disagree completely on
traction: at 30 % slip treaded SCM gives about 1650 N of drawbar pull on sand and CRM about
-100 N. The doc's "Still to measure" item says what is missing: a measured drawbar test of a
real road tire on each soil, which is what decides where the truth lies.

So the question for each class is: which published measurements give pressure-sinkage and
shear parameters, cone index, and above all drawbar pull (or traction coefficient), motion
resistance and sinkage against slip for a **pneumatic** tire, ideally a passenger-car or
light-truck tire at 4-5 kN? Rover and rigid-wheel work is recorded only as secondary, because
a rigid 0.2 m wheel at 50 N says little about a deflecting road tire at 4.5 kN.

For each source the tables give the citation, the soil and its state, the tire, what was
measured, the format, the access, and how usable it is for us. "Unverified" marks anything
that came from an abstract, an index or a secondary reproduction rather than the source
itself. Access was checked on 2026-10-24: DTIC (`apps.dtic.mil`) and the ERDC library
(`erdc-library.erdc.dren.mil`) both serve public-domain reports but block scripted downloads
(403 and a reCAPTCHA page respectively), so their contents are listed from catalogue
records; a browser gets them.

One finding runs through every section: **the passenger-car tire on soft soil is almost
absent from the open literature.** Military mobility work (WES, CRREL, NG-NRMM) used truck,
military and agricultural tires; planetary work uses rigid wheels. The one clear exception is
Virginia Tech's Terramechanics Rig with the ASTM Standard Reference Test Tire (SRTT,
P225/60R16), which is a passenger tire within a size of ours, at 5 kN.

## Dry sand (`sand`)

| source | soil and state | tire | measured | format | access | use |
|---|---|---|---|---|---|---|
| **He, R. (2020)**, *Systematic Tire Testing and Model Parameterization for Tire Traction on Soft Soil*, PhD dissertation, Virginia Tech. [hdl.handle.net/10919/104386](http://hdl.handle.net/10919/104386). Journal versions: He, Sandu, Osorio, J. Terramechanics 85 (2019) 59-76 (Part I, data) and 77-88 (Part II, parameterization), [doi:10.1016/j.jterra.2019.07.003](https://doi.org/10.1016/j.jterra.2019.07.003) | loamy sand (USDA; 84 % sand, 16 % silt, 0 % clay), kept at its driest: gravimetric water 0.4-1.0 %, VWC 0.9-1.5 %; dry density 1.72-1.87 g/cm^3 (89-97 % of Proctor); three compaction levels, CI ~250, ~700, ~980 kPa | **225/60R16 97S Uniroyal SRTT**, 5 kN (free-rolling also 6 and 7 kN), 124 / 207 / 290 kPa (18 / 30 / 42 psi) | drawbar pull coefficient, drawbar efficiency, sinkage, rut depth/width/length, contact-patch length and pressure (pad), vs. slip 0-70 % in 14 steps; multi-pass; post-traffic CI and density; Bekker n, k_c, k_phi inverted from static ruts; Magic Formula fits of DP(slip) per inflation | figures for the curves; numeric tables for soil state, the design of experiment, the Bekker inversion (Table 5.8) and Magic Formula coefficients (Tables 5.3-5.6); no raw data file | dissertation open (VTechWorks PDF, read here); papers paywalled | **high**: a passenger tire within one size of ours, at our load and inflation, with slip sweeps. Read directly: the MF peak D at low compaction is 1.65 / 1.01 / 0.60 kN at 18 / 30 / 42 psi, a peak DP coefficient of 0.33 / 0.20 / **0.12**, at about 12 % slip regardless of inflation. Limits: carriage speed 6 cm/s; no shear-annulus test, so no c, phi, K; its Bekker fit (n 0.22-0.31) comes from rut inversion and its units were not checked |
| **Naranjo, Sandu, Taheri, Taheri (2014)**, "Experimental testing of an off-road instrumented tire on soft soil", J. Terramechanics 56, 119-137, [doi:10.1016/j.jterra.2014.09.003](https://doi.org/10.1016/j.jterra.2014.09.003); thesis [hdl 10919/51165](https://vtechworks.lib.vt.edu/handle/10919/51165) | the same loamy sand (He says so), several compactions | an off-road tire with a wireless deflection sensor; size unverified | normalized DP and sinkage vs. slip, load, inflation, compaction; sinkage 3-4x higher at 70-90 % slip than at 0-5 % | plots | paper paywalled, thesis open | medium: same rig and soil as He, an off-road rather than road tire |
| **DROVE** (Database Records for Off-road Vehicle Environments), ERDC with Mississippi State: Vahedifard et al., J. Terramechanics 63 (2016) 13-22, [doi:10.1016/j.jterra.2015.10.002](https://doi.org/10.1016/j.jterra.2015.10.002); Mason et al., unified traction equation, J. Terramechanics 79 (2018) 33-40, [doi:10.1016/j.jterra.2018.05.005](https://doi.org/10.1016/j.jterra.2018.05.005); Williams et al., motion resistance, J. Defense M&S 16(3) (2019) 283-295, [doi:10.1177/1548512917693119](https://doi.org/10.1177/1548512917693119); [EasyChair preprint 1400](https://easychair.org/publications/preprint/R72F) (2019 status) | loose dry sands (Yuma, mortar and others), CI ~50-800 kPa | 23 wheel configurations in the sinkage set, diameters 0.36-1.24 m, loads 0.19-36 kN; the motion-resistance set 0.19-4.49 kN, 0.38-1.05 m: our tire (0.67 m, 4.5 kN) is inside, at its upper load edge | DP, motion resistance, torque, sinkage, slip; 8000+ records in DROVE 1.0, digitised from WES reports | closed-form equations in the papers (pressure/CI ratio and slip); the database itself was not found published | equations paywalled; database: no public release found, access on request to ERDC (unverified) | **high** as a fallback: the WES pneumatic-tire record, condensed into DP(slip) and MR(slip) laws with cone index as the soil input, which 040's field cone index can feed directly |
| **WES TR 3-666, *Performance of Soils under Tire Loads*** (US Army Waterways Experiment Station, 1960s-70s): Report 2 (Powell, Yuma sand), Report 4 (Turnage and Green 1966, sand tests 1962-63), Report 5 (mobility numbers, coarse soils), Report 8 (Turnage 1972, tire selection). Catalogue: [TRID](https://trid.trb.org/View/1169994), [ERDC](https://erdc-library.erdc.dren.mil/jspui/handle/11681/31440) | Yuma and mortar sands, by cone index | military and agricultural tires, e.g. 9.00-14 and 11.00-20 (unverified per report) | pull, torque, sinkage at fixed slips (often 20 %), multi-pass | tables and plots in scanned reports (WES reports usually tabulate each test; unverified) | public domain (US government), DTIC/ERDC, browser only | medium: the raw source behind DROVE; heavy tires, but numeric per-test data on sand |
| **Cranfield Single Wheel Tester**, "An Investigation into Wheel Sinkage on Soft Sand", Tire Science and Technology 42(2) (2014) 85-100, [doi:10.2346/tire.14.420201](https://doi.org/10.2346/tire.14.420201) (authors not recorded here) | dry desert-like sand, Cranfield soil lane | Goodyear G90 (a light-truck/military size; unverified), 1-5 kN, 69-276 kPa | sinkage vs. load, inflation and slip; slip swept down from 85 % | plots (unverified) | paywalled | medium: pneumatic, our load range, sinkage only (traction unverified) |
| **Wong, *Theory of Ground Vehicles*** (Wiley, 3rd ed. 2001 / 4th ed. 2008), Table 2.3 | "dry sand (Land Locomotion Lab)", n 1.1, k_c 0.99 kN/m^(n+1), k_phi 1528.43 kN/m^(n+2), c 1.04 kPa, phi 28 deg; also LETE sand (Wong's own measurement) | none (plate and shear-annulus tests) | bevameter parameters; Janosi K only as typical ranges in the text | numeric table | book (paywalled); the row is reproduced throughout the literature | medium: the source of `sand`'s row; parameters, not a tire test |
| Chrono SCM/CRM calibration papers: Hu et al. [arXiv 2507.05643](https://arxiv.org/html/2507.05643v2); SBEL, "Using high-fidelity DEM to calibrate an expeditious terramechanics model", [arXiv 2407.18903](https://arxiv.org/abs/2407.18903) | simulated soils; the experiments they compare with are planetary single-wheel tests (e.g. MGRU3, GRC-1 simulant) | rigid grousered rover wheels | DP and sinkage vs. slip | scripts open (repository named in the paper, contents unverified); the physical data are the original lab's | open papers | low: sim-to-sim, or rigid rover wheels; no released pneumatic-tire data found |

## Dirt / sandy loam (`dirt`)

| source | soil and state | tire | measured | format | access | use |
|---|---|---|---|---|---|---|
| **NG-NRMM Cooperative Demonstration of Technology (CDT)**, NATO AVT-248, Keweenaw Research Center, 25-27 Sep 2018. Data paper: Gerth et al., NDIA GVSETS 2019 draft, [mtu.edu/cdt](https://www.mtu.edu/cdt/) (`p-m-draftpaper-cdt-data-gerth-richard.docx`, read here); [CDT technical memorandum](https://www.mtu.edu/cdt/downloadable-materials/cdt-technical-memorandum.pdf); final report [STO-TR-AVT-248](https://www.sto.nato.int/publications/STO%20Technical%20Reports/STO-TR-AVT-248/$TR-AVT-248-ES.pdf) (2020) | five soil regimes, each with lab and in-situ tests: Fine Grained Pit (ML sandy silt; stable at 6 % moisture, "essentially no strength" at 18 %), Coarse Grained Pit (SP/SM rounded medium sand), 2NS sand (SP, angular), Rink Natural (SM silty sand, cohesive), Stability (SW/SM well-graded silty sand). Lab: grain size, Proctor, Atterberg, direct and triaxial shear. In situ: cone traces, nuclear density and moisture, **plate sinkage and shear annulus (bevameter) at every test area**, raw and reduced | FED-Alpha, ~12 t, **Goodyear 335/65R22.5** G275 with CTIS 15/25/40/60 psi | **drawbar pull vs. slip** on fine-grained organic silty sand wet and dry, and coarse sand dry; sand grades; plus paved handling events | raw vehicle channels in .mat, summaries in Excel/PowerPoint; raw and reduced bevameter data; GIS layers (GeoTIFF, geodatabase) | **Distribution A, unlimited**: published on KRC's anonymous FTP (credentials printed in the public access sheet, `data-set-access-info.pdf`); whether the server still answers was not checked | **high for the soil parameters**: the only open set found with raw bevameter data (k_c, k_phi, n, c, phi, K) on sandy soils at known moisture beside a drawbar test. Low for the tire: a truck tire at ~25-30 kN per wheel |
| **He (2020)** at medium and high compaction (see sand) | loamy sand, CI ~700 and ~980 kPa, dry | SRTT 225/60R16, 5 kN | as above; DP coefficient, its initial slope and asymptote all rise with compaction | as above | open | **high** for the tire: the only passenger-tire DP(slip) found; but the soil is dry and nearly cohesionless, where our `dirt` is a moist, cohesive loam |
| **Wong, Table 2.3** | sandy loam (LLL) at 15 % moisture: n 0.7, k_c 5.27, k_phi 1515.04, c 1.72 kPa, phi 29 deg (our `dirt`); also LLL sandy loam at other moisture, Michigan sandy loam (Strong, Buchele) at 11 and 23 %, and Wong's own Upland sandy loam, Rubicon sandy loam, North Gower clayey loam, Grenville loam (row names from memory of the book; only the row in `surface_classes.yaml` matches the literature's reproductions, the rest unverified) | none | bevameter parameters | numeric table | book | medium: shows how strongly moisture moves the parameters for one soil |
| Lee and Gard (2014), "Vehicle-soil interaction: testing, modeling, calibration and validation", J. Terramechanics 52, 9-21 ([ISTVS listing](https://www.istvs.org/publication-news/2014/1/15/vehiclesoil-interaction-testing-modeling-calibration-and-validation)); Lee (2015), combined slip, J. Terramechanics 58, 11-25 | sand (keyword), field; state unverified | the University of Alaska Fairbanks instrumented vehicle (a road vehicle; tire unverified) | DP and torque vs. slip, combined slip in 2015 | plots and metamodels | paywalled | medium: a vehicle on road-type tires in the field; soil and tire to be checked in the full text |
| DROVE fine-grained component: Williams et al., "Development of a multi-year database to assess off-road mobility algorithms in fine-grained soils", Int. J. Vehicle Performance (2017), [Inderscience 81259](https://www.inderscience.com/offer.php?id=81259) | fine-grained soils, 2657 tests | WES tires | DP, MR, torque, sinkage, slip | equations; database not public | paywalled | medium: the cohesive-soil counterpart of the sand equations |

## Mud / wet clay (`mud_wet`)

| source | soil and state | tire | measured | format | access | use |
|---|---|---|---|---|---|---|
| **WES TR 3-666 Report 3**, Wismer, R. D. (1966), *Tests in Clay through November 1962*, [ERDC 11681/31440](https://erdc-library.erdc.dren.mil/jspui/handle/11681/31440) | **fat clay (CH) at a high degree of saturation**, by cone index | single smooth pneumatic tire, 9.00-14, 2-ply | 184 multi-pass tests: pull, torque, sinkage, slip | scanned report, tables likely (unverified) | public domain, browser only | **high relative to anything else**: a pneumatic tire on near-saturated clay with cone index, the closest measured thing to "mud"; the tire is narrower and softer than ours |
| WES report on tread effect on wet clay ([DTIC AD0747099](https://apps.dtic.mil/sti/pdfs/AD0747099.pdf); title unverified) | strong fat clay with a **wet surface**, the slippery-film case | treaded vs. smooth single tires, 20 % slip and locked | traction and skid resistance | report | public domain | medium: the slick-surface mechanism our friction-only `mud_wet` represents, as opposed to deep soft mud |
| **NG-NRMM CDT** Fine Grained Pit (see dirt) | ML sandy silt up to 18 % moisture, "essentially no strength", with bevameter and cone data | FED-Alpha 335/65R22.5 | DP vs. slip, "fine grain organic/silty sand - wet" | .mat, Excel, bevameter | Distribution A, FTP | medium: wet, cohesive and open, with measured parameters; a truck |
| **Wong, Table 2.3** | clayey soil (Thailand) at 38 % and 55 % moisture (55 %: n 0.7, k_c 16.03, k_phi 1262.53, c 2.07 kPa, phi 10 deg, our `mud_wet`); heavy clay (WES) at 25 and 40 %; lean clay (WES) at 22 and 32 % (other rows' values not checked) | none | bevameter parameters | numeric table | book | medium: the WES clay rows come from the same institution as Report 3, so a pairing may be possible |
| Wong's muskeg (mat and peat) characterisation, *Terramechanics and Off-Road Vehicle Engineering* (2nd ed. 2010) and Wong, Radforth, Preston-Thomas (1982), J. Terramechanics 19(2) (unverified) | saturated organic terrain | tracked and wheeled vehicles | pressure-sinkage of the mat-peat model (a different model form) | tables | paywalled | low: a soft wet terrain with real measurements, but not Bekker's form |
| Sedara and Manuwa (2022), CIGR Journal, motion resistance of a single narrow wheel on clay ([cigrjournal.org](https://cigrjournal.org/index.php/Ejounral/article/download/6379/3795/36075)) | clay soil, several moistures | small pneumatic tire, several inflations | motion resistance | plots/tables | open | low: small wheel, resistance only |
| DROVE clay, Mason et al., "A unified equation for predicting gross traction for wheels on clay ..." ([ISTVS listing](https://istvs.org/publication-news/a-unified-equation-for-predicting-gross-traction-for-wheels-on-clay-over-a-range-of-braked-towed-and-powered-operations); year unverified) | high-plasticity clay | WES tires | gross traction vs. slip, from CI | equation | paywalled | medium: a closed-form fallback for clay at a given CI |

Nothing found measures a road tire in wet clay with the pore-water, rate and clogging effects
D6 already lists as missing.

## Snow, fresh and packed (`snow_loose`, `snow_packed`)

| source | snow | tire | measured | format | access | use |
|---|---|---|---|---|---|---|
| **Lee, Huang, Johnson, Meurer, Reid, Meldrum (2012)**, "Slip-based experimental studies of a vehicle interacting with natural snowy terrain", J. Terramechanics 49(5), 233-244, [doi:10.1016/j.jterra.2012.07.002](https://doi.org/10.1016/j.jterra.2012.07.002) | natural snow, Alaska; strength and depth by SnowMicroPen, sinkage by laser profilometer; also the rear wheel on snow the front wheel compacted | the University of Alaska Fairbanks instrumented vehicle; tire size, load and inflation unverified | **DP, motion resistance, wheel states and tire stiffness vs. slip** over a wide slip, load and torque range; peak normalized traction about 0.47, peak normalized motion resistance about 0.4 | plots | paywalled | **high**: a road-type vehicle on fresh snow with slip sweeps and snow characterisation. Follow-ups: Lee (2013) calibration, J. Terramechanics 50(5-6) 289-302, [doi:10.1016/j.jterra.2013.09.003](https://doi.org/10.1016/j.jterra.2013.09.003); Lee and Huang (2014) combined slip; "Vehicle-wet snow interaction" (ISTVS listing, details unverified); the 2009 TARDEC briefing [DTIC ADA517668](https://apps.dtic.mil/sti/pdfs/ADA517668.pdf) |
| **Shoop, S. (1993)**, *Three Approaches to Winter Traction Testing*, CRREL Report 93-9, [DTIC ADA276484](https://apps.dtic.mil/sti/pdfs/ADA276484.pdf); SAE 940110 (Shoop, Young, Alger, Davis 1994) | packed snow and ice test surfaces (details unverified) | CRREL Instrumented Vehicle (CIV), a Uniroyal-Goodrich commercial tire-traction tester (passenger tires, likely), a Saab friction tester | traction coefficients by three methods; comparable with systematic differences | report with tables (unverified) | **public domain**, browser only | **high for packed snow and ice**: winter traction of road-type tires on prepared surfaces |
| **Richmond, P. W. (1995)**, *Motion Resistance of Wheeled Vehicles in Snow*, CRREL Report 95-7 ([ROSAP copy](https://rosap.ntl.bts.gov/view/dot/14081/dot_14081_DS1.pdf)) | shallow fresh snow, depth and density | CIV and military vehicles | motion resistance; a trailing CIV tire at about 0.017 for snow under ~22 cm | report | public domain, open (ROSAP) | medium: the resistance half of fresh snow, in a form the grid's zero-slip pull can be checked against |
| **Blaisdell, Richmond, Shoop, Green, Alger (1990)**, *Wheels and Tracks in Snow: Validation Study of the CRREL Shallow Snow Mobility Model*, CRREL Report 90-9 ([TRID](https://trid.trb.org/View/349154)); second validation study ([TRID](https://trid.trb.org/View/350037)); Richmond, Blaisdell, Green, CRREL 90-13 (HMMWV, HEMTT, SUSV, M60 and CIV traction on hard-packed snow and on snow over ice) | shallow snow, measured depth, density, strength | military vehicles and the CIV | traction and resistance coefficients | reports | public domain | medium: the empirical shallow-snow model NRMM uses, with its validation data |
| **Shoop, S. (2001)**, *Finite Element Modeling of Tire-Terrain Interaction*, ERDC/CRREL TR-01-16, [ERDC 11681/5442](https://erdc-library.erdc.dren.mil/jspui/handle/11681/5442) | fresh snow and compacted sand, critical-state plasticity | rigid, simplified and modal FE tires, validated with CIV force measurements and measured snow deformation | forces, sinkage | report | public domain | medium: CIV fresh-snow measurements as validation targets, and a statement that a road tire barely deflects on fresh snow (so a rigid-wheel check is fair there) |
| **Wong, Table 2.3, snow rows** (see below) | snow (U.S.), snow (Sweden) | none | bevameter parameters | numeric table | book | medium |
| Mundl, Meschke, Liederer (1997), "Friction mechanism of tread blocks on snow surfaces", Tire Sci. Technol. 25(4) (unverified) | snow, lab | tread blocks | friction | plots | paywalled | low: tread-block scale |
| Hallonborg (1996), "Super ellipse as tyre-ground contact area", J. Terramechanics 33(3) (unverified) | soft ground, forestry | forestry tires | contact shape | plots | paywalled | low |
| SAE J1466 (passenger and light-truck tire traction in snow), ASTM F1805 (single-wheel driving traction on snow, using the SRTT as reference), ASTM F1572 | standardised medium-pack snow | passenger and LT tires, SRTT | traction indices relative to the SRTT | standards; results proprietary | paywalled | low: methods, no open numbers; but F1805 uses the same SRTT as He (2020), so a published F1805 traction value for the SRTT would tie snow to sand on one tire |

"Kotulski" from the candidate list was not found as a snow-traction author; it is left out.

### Wong's snow rows and `snow_loose`

Wong's Table 2.3 does have snow: two rows, as the book is reproduced in the literature (not
checked against the book here, the same caveat 047 D6 gives for the clay row):

| | n | k_c, kN/m^(n+1) | k_phi, kN/m^(n+2) | c, kPa | phi, deg |
|---|---|---|---|---|---|
| snow (U.S.) | 1.6 | 4.37 | 196.72 | 1.03 | 19.7 |
| snow (Sweden) | 1.44 | 10.55 | 66.08 | 6.0 | 20.7 |
| `snow_loose` in surface_classes.yaml | 1.6 | 4370 N | 196720 N | 1030 Pa | 19.7 |

`snow_loose` is exactly the U.S. row in SI units; its Janosi K (0.04 m) is not from the table
but from the book's typical ranges, and nothing measured backs its slip sinkage (left at 1).
The book does not give density, depth or age for either row, so "loose" is our reading of
"U.S.", not the source's. The Swedish row, with six times the cohesion, behaves like a firmer
snow and is the obvious candidate if `snow_packed` ever needs a soil zone; today it is friction
only (mu 0.2, Wong's Table 1.1, below). Neither row is tied to a tire test.

## Ice and wet asphalt (friction only)

| source | surface | tire | measured | format | access | use |
|---|---|---|---|---|---|---|
| **Schleinin, Savitski, Jimenez, Sandu, Ivanov (2017)**, *Dataset of tyre tests on ice - Part 1*, Zenodo, [doi:10.5281/zenodo.1009017](https://doi.org/10.5281/zenodo.1009017) | ice on Virginia Tech's Terramechanics Rig | not stated on the record (the rig's ice work used the SRTT: Bhoopalam and Sandu 2013; Mousavi and Sandu 2020, J. Terramechanics) | tyre tests on ice; 44 MB zip | raw data files | **CC BY 4.0, open** | **high** for `ice`: the only open raw tire-on-ice data found; contents not opened here |
| Shoop (1993), CRREL 93-9 (see snow) | ice | as above | traction coefficients | report | public domain | medium |
| **Wong, *Theory of Ground Vehicles*, Table 1.1** (peak and sliding tire-road friction by surface) | asphalt and concrete dry and wet, gravel, earth road dry and wet, packed snow, ice | generic | peak and sliding mu | numeric table | book | medium: the classes' `mu` values (asphalt 0.8-0.9, gravel 0.6, dry earth 0.68, packed snow 0.2, ice 0.1) match this table, which is a handbook summary, not a test |
| Experimentally measured mu-slip curves for many tire/road pairs, dry, wet and glare ice, SAE 690214 (1969) ([SAE](https://saemobilus.sae.org/content/690214)) | road surfaces and ice | commercial passenger tires | mu vs. slip | plots | paywalled | medium: the curve shapes the `slip:` graphs approximate |

Wet asphalt: nothing open found beyond handbook factors and test-surface specifications (wet
skid-pad friction levels in braking regulations, measured with the SRTT under ASTM E1337;
unverified). Not pursued further.

## Recommended reference per class

**Dry sand.** Primary: **He (2020)**, the SRTT on dry loamy sand at low compaction. It is the
only passenger tire at our load found with slip sweeps, the dissertation is open, and it can
be reproduced in our grid directly: the SRTT is a 0.67 m, 225 mm tire at 5 kN, near the S5 rig
wheel. Its 42 psi peak drawbar pull coefficient of 0.12 at about 12 % slip lies between
treaded SCM's (about 0.3) and CRM's (below zero), which is the disagreement S5 left open. Two
cautions: the soil is a loamy sand at CI ~250 kPa, not Wong's "dry sand", so the comparison
needs our grid run with soil parameters matching He's soil (from his CI and density, or a CI
conversion), not `sand`'s row; and the carriage ran at 6 cm/s. Fallback: the **DROVE
equations** (Mason 2018 traction, Williams 2019 motion resistance, Mason 2016 sinkage), which
cover our load and diameter and take cone index as input, so 040's field cone index can be
fed straight in.

**Dirt / sandy loam.** Primary: the **NG-NRMM CDT soil data** for the parameters: raw
bevameter plate-sinkage and shear-annulus data, cone traces, density and moisture on the
Rink Natural and Stability silty sands, Distribution A. It is the only open set that measures
every parameter a class carries, including Janosi K, on real moist cohesive-ish soils. For the
tire behaviour, use He (2020) at medium and high compaction, accepting the dry soil.
Fallback: Wong's sandy-loam rows across moisture, for the trend, with Lee and Gard (2014)
once its soil and tire are confirmed.

**Mud / wet clay.** Primary: **WES TR 3-666 Report 3** (Wismer 1966), 184 tests of a
pneumatic tire on near-saturated fat clay with cone index, public domain. It does not make
`mud_wet` validated (wrong tire, a lab clay, no rate effects), but it is the only measured
pneumatic traction on saturated clay found, and it pairs with Wong's WES heavy- and lean-clay
rows from the same laboratory. Fallback: the CDT **Fine Grained Pit at high moisture**, with
its bevameter data and the FED-Alpha's wet drawbar pull, open but on a truck tire. The
`unvalidated:` flag should stay either way.

**Snow, fresh.** Primary: **Lee et al. (2012)**: a road vehicle on natural snow, DP and
motion resistance against slip, snow characterised by micropenetrometer; paywalled, and the
tire must be confirmed from the full text. Fallback: the **CRREL reports** (Richmond 1995 for
resistance, Blaisdell et al. 1990 and Shoop 2001 for traction and validation forces), public
domain.

**Snow, packed.** Primary: **CRREL 93-9** (Shoop 1993), winter traction on prepared packed
snow and ice from three instruments including a commercial tire tester, public domain.
Fallback: Wong's Table 1.1 for mu and the Swedish snow row for a soil zone.

**Ice.** Primary: the **Zenodo Virginia Tech ice dataset**, CC BY 4.0 raw data. Fallback:
CRREL 93-9.

## Gaps

- **A passenger tire on moist cohesive soil, measured.** He's soil was kept at about 1 %
  moisture because the rig's bin could not hold more. Nothing open measures a road tire's
  drawbar pull on a sandy loam at 10-20 % moisture, which is what `dirt` claims to be.
- **Mud.** No pneumatic road-tire test on wet clay or mud with slip sweeps, and nothing on
  rate dependence, suction or tread clogging. The WES clay work is narrow, soft military
  tires at walking pace.
- **Speed.** Every single-wheel test found runs at centimetres per second (He: 6 cm/s). The
  soil layer is driven at 10-20 m/s. Field vehicle tests (CDT, Lee, CRREL) are faster but
  give whole-vehicle or per-wheel data under less control. No public data shows how drawbar
  pull or sinkage change with speed on soft soil for a road tire.
- **Shear parameters with a tire test on the same soil.** Janosi K, the number S5 fitted
  hardest, is measured only by shear annulus. He has no annulus test, the CDT has annulus
  data but a truck tire, and Lee's snow work uses a micropenetrometer. No open source gives
  annulus K and a road-tire DP(slip) on one soil.
- **Raw numbers.** Apart from the CDT (truck) and the Zenodo ice set, the tire curves exist
  only as plots or fitted Magic Formula coefficients. DROVE, which digitised the WES record,
  has not been published as a dataset as far as could be found. A request to ERDC or to
  Virginia Tech's TMVS lab (He's raw data) is the cheapest way to get numbers.
- **Snow state.** Wong's two snow rows carry no density, depth or temperature, so neither can
  be matched to a particular snow; the field snow papers measure state but not Bekker
  parameters.
- **Not verified here**: Wong's table values beyond the rows already in
  `surface_classes.yaml`; the contents of every DTIC/ERDC report (catalogue records only);
  the tires of Naranjo (2014), Lee (2012) and Lee and Gard (2014); whether KRC's CDT FTP is
  still online; the units of He's Bekker inversion.
