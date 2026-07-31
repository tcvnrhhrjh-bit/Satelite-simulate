%% RIoT-2 orbit, telemetry visualization, and digital-twin simulation
% New version for 2026-07-08 telemetry archive.
% It keeps the original orbit / ground-station workflow, then adds:
% 1) thermal cycle versus eclipse,
% 2) RSSI versus Zhongli range/elevation,
% 3) telemetry correlation analysis,
% 4) first-order thermal prediction,
% 5) simplified power budget / battery SOC prediction.

clear; clc; close all;

%% 1. Paths and user options
thisFile = mfilename("fullpath");
outDir = fileparts(thisFile);
rootDir = fileparts(outDir);

archiveCandidates = string([ ...
    fullfile(outDir, "Archive(1)"), ...
    fullfile(rootDir, "Archive(1)"), ...
    "C:\Users\USER\OneDrive\桌面\實習\7-8\Archive(1)" ...
    ]);
archiveDir = "";
for c = archiveCandidates
    if isfolder(c)
        archiveDir = c;
        break;
    end
end
if archiveDir == ""
    archiveDir = findFirstTelemetryArchive([outDir, rootDir, ...
        "C:\Users\USER\Documents\Codex\2026-07-07\new-chat\outputs\multi_day_archive", ...
        "C:\Users\USER\Documents\Codex\2026-07-07\new-chat\work\archive_extracted"]);
end

tleFile = fullfile(outDir, "RIoT2_20260706.tle");
fallbackRoot = "C:\Users\USER\Documents\Codex\2026-07-07\tle-matlab";
fallbackTleFile = fullfile(fallbackRoot, "outputs", "RIoT2_20260706.tle");
if ~isfile(tleFile)
    tleFile = fallbackTleFile;
end

show3DViewer = false;    % Set true to open the 3D satellite scenario viewer.
sampleTimeSeconds = 60;  % Satellite scenario time step.
forcePopupFigures = true; % Use separate figure windows instead of cramped Live Editor output.
openExportedFigureWindows = false; % Export white-background PNGs without opening many image windows.
closeFiguresAfterExport = true;   % Prevent dark Live Editor inline figure output from covering the white plots.
exportPortfolioReport = true;     % Write CSV/Markdown deliverables for a portfolio-style submission.

% Body-frame assumptions for pointing analysis. Tune these if the spacecraft
% CAD/model uses different mission axes.
nadirBoresightBody = [0; 0; 1];
solarArrayNormalBody = [1; 0; 0];
attitudeModeThresholdDeg = 25;
figureOutDir = fullfile(outDir, "figure_outputs");
if ~isfolder(figureOutDir)
    mkdir(figureOutDir);
end

if forcePopupFigures
    set(groot, "DefaultFigureWindowStyle", "normal");
end
set(groot, "DefaultFigureColor", "w", ...
    "DefaultAxesColor", "w", ...
    "DefaultAxesFontName", "Arial", ...
    "DefaultAxesFontSize", 12, ...
    "DefaultAxesLineWidth", 1.0, ...
    "DefaultAxesXColor", [0.12 0.12 0.12], ...
    "DefaultAxesYColor", [0.12 0.12 0.12], ...
    "DefaultTextColor", [0.08 0.08 0.08], ...
    "DefaultLineLineWidth", 1.8);

if ~isfile(tleFile)
    error("Cannot find TLE file. Put RIoT2_20260706.tle beside this MLX or check: %s", fallbackTleFile);
end
if archiveDir == ""
    error("Cannot find telemetry archive folder. Checked: %s", strjoin(archiveCandidates, ", "));
end

%% 2. Read and clean telemetry CSV files
hkFiles = dir(fullfile(archiveDir, "*-RIoT-2-hk.csv"));
hkFiles = hkFiles(~startsWith({hkFiles.name}, "._"));
if isempty(hkFiles)
    error("No RIoT-2 HK CSV files found in: %s", archiveDir);
end

hk = readAndStackTelemetry(hkFiles);
hkTime = telemetryTime(hk);
[hkTime, sortIdx] = sort(hkTime);
hk = hk(sortIdx, :);

validTime = ~isnat(hkTime);
hk = hk(validTime, :);
hkTime = hkTime(validTime);

[hkTime, uniqueIdx] = unique(hkTime, "stable");
hk = hk(uniqueIdx, :);

fprintf("Loaded %d telemetry rows from %d CSV files.\n", height(hk), numel(hkFiles));
fprintf("Telemetry span: %s to %s UTC.\n", string(hkTime(1)), string(hkTime(end)));

%% 3. Build the satellite scenario from TLE
sc = satelliteScenario(hkTime(1), hkTime(end), sampleTimeSeconds);
sat = satellite(sc, tleFile, "Name", "RIoT2");

try
    sat.Visual3DModel = "SmallSat.glb";
    sat.Visual3DModelScale = 1;
catch
    warning("SmallSat.glb was not available. Continuing with the default satellite marker.");
end

gs = groundStation(sc, 24.96, 121.22, "Name", "Zhongli_GS");
tx = transmitter(sat, "Frequency", 2.4e9, "Power", 10);
rx = receiver(gs, "Name", "Zhongli_Rx");
link(tx, rx);
ac = access(sat, gs);

accessVector = accessStatus(ac);
fprintf("Access samples with Zhongli ground station: %d of %d\n", nnz(accessVector), numel(accessVector));

%% 4. Reconstruct attitude from telemetry quaternion, when available
qNames = ["iEstimatedORCquaternionQ0", "iEstimatedORCquaternionQ1", ...
          "iEstimatedORCquaternionQ2", "iEstimatedORCquaternionQ3"];
validQ = false(height(hk), 1);
RBodyToEciFlat = nan(height(hk), 9);
if all(hasVar(hk, qNames))
    qBodyOrc = double([getVar(hk, qNames(1)), getVar(hk, qNames(2)), ...
                       getVar(hk, qNames(3)), getVar(hk, qNames(4))]) * 1e-4;
    qNorm = vecnorm(qBodyOrc, 2, 2);
    validQ = all(isfinite(qBodyOrc), 2) & qNorm > 0;
    qBodyOrc(validQ, :) = qBodyOrc(validQ, :) ./ qNorm(validQ);

    qInertial = nan(height(hk), 4);
    for k = find(validQ).'
        [rk, vk] = states(sat, hkTime(k), "CoordinateFrame", "inertial");
        R_eci_from_orc = eciFromLvlh(rk(:), vk(:));
        R_orc_from_body = quatScalarLastToDcm(qBodyOrc(k, :));
        R_eci_from_body = R_eci_from_orc * R_orc_from_body;
        RBodyToEciFlat(k, :) = reshape(R_eci_from_body, 1, []);
        qInertial(k, :) = dcm2quat(R_eci_from_body.');
    end

    attitudeTT = timetable(hkTime(validQ), qInertial(validQ, :));
    pointAt(sat, attitudeTT);
else
    warning("Quaternion columns were not found. The orbit analysis will continue without attitude playback.");
end

%% 5. Orbit-derived features: eclipse, elevation, range, and sun angle
n = height(hk);
eclipseFlag = false(n, 1);
sunlitFlag = false(n, 1);
sunIncidenceProxy = nan(n, 1);
elevationDeg = nan(n, 1);
azimuthDeg = nan(n, 1);
rangeKm = nan(n, 1);
rEciUnit = nan(n, 3);
sunHatEci = nan(n, 3);

for k = 1:n
    [rk, ~] = states(sat, hkTime(k), "CoordinateFrame", "inertial");
    rk = rk(:);
    sunHat = approximateSunEciUnit(hkTime(k));
    rEciUnit(k, :) = (rk ./ norm(rk)).';
    sunHatEci(k, :) = sunHat.';
    eclipseFlag(k) = isInCylindricalEclipse(rk, sunHat);
    sunlitFlag(k) = ~eclipseFlag(k);
    sunIncidenceProxy(k) = max(0, dot(rk ./ norm(rk), sunHat)) * double(sunlitFlag(k));

    try
        [azimuthDeg(k), elevationDeg(k), rangeMeters] = aer(gs, sat, hkTime(k));
        rangeKm(k) = rangeMeters / 1000;
    catch
        [elevationDeg(k), rangeKm(k), azimuthDeg(k)] = deal(nan);
    end
end

%% 5-1. Portfolio extension: attitude pointing error and inferred ADCS mode
nadirPointingErrorDeg = nan(n, 1);
sunPointingErrorDeg = nan(n, 1);
inferredAdcsMode = strings(n, 1);
inferredAdcsMode(:) = "no_quaternion";

for k = find(validQ).'
    R_eci_from_body = reshape(RBodyToEciFlat(k, :), 3, 3);
    bodyNadirAxisEci = R_eci_from_body * nadirBoresightBody;
    bodySolarAxisEci = R_eci_from_body * solarArrayNormalBody;

    nadirTargetEci = -rEciUnit(k, :).';
    sunTargetEci = sunHatEci(k, :).';

    nadirPointingErrorDeg(k) = angularSeparationDeg(bodyNadirAxisEci, nadirTargetEci);
    if sunlitFlag(k)
        sunPointingErrorDeg(k) = angularSeparationDeg(bodySolarAxisEci, sunTargetEci);
    end
    inferredAdcsMode(k) = classifyAttitudeMode( ...
        nadirPointingErrorDeg(k), sunPointingErrorDeg(k), sunlitFlag(k), attitudeModeThresholdDeg);
end

fprintf("\nAttitude pointing checks:\n");
fprintf("valid quaternion coverage    = %.1f %%\n", 100 * mean(validQ));
fprintf("median nadir pointing error  = %.2f deg\n", median(nadirPointingErrorDeg, "omitnan"));
fprintf("median sun pointing error    = %.2f deg\n", median(sunPointingErrorDeg, "omitnan"));

%% 6. Telemetry channels used in the analysis
spTemp = meanAvailable([getVar(hk, "sp_temp[0]"), getVar(hk, "sp_temp[1]"), getVar(hk, "sp_temp[2]")]);
tempMcu = scaleMaybeTenthsC(getVar(hk, "temp_mcu"));
tempBoard = scaleMaybeTenthsC(getVar(hk, "temp_board"));
lastRssi = getVar(hk, "last_rssi");
obcMode = getVar(hk, "current_obc_mode");

battVolt = firstAvailable([ ...
    getVar(hk, "vbat_mV") / 1000, ...
    getVar(hk, "PDU1vbat_mV") / 1000, ...
    getVar(hk, "PDU2vbat_mV") / 1000, ...
    getVar(hk, "BPX1vbatt") / 1000, ...
    getVar(hk, "BPX2vbatt") / 1000]);

solarPowerW = sumAvailable([ ...
    getVar(hk, "power_mW[0]"), getVar(hk, "power_mW[1]"), ...
    getVar(hk, "power_mW[2]"), getVar(hk, "power_mW[3]"), ...
    getVar(hk, "power_mW[4]"), getVar(hk, "power_mW[5]")]) / 1000;

%% 7. Data visualization and correlation analysis
figThermal = readableFigure("Thermal cycle versus eclipse", [80 70 1180 720]);
tlThermal = tiledlayout(figThermal, 2, 1, "TileSpacing", "compact", "Padding", "compact");
title(tlThermal, "Thermal telemetry versus orbit eclipse", "FontSize", 18, "FontWeight", "bold");
ax = nexttile(tlThermal);
plot(hkTime, spTemp, "Color", [0.90 0.42 0.10]); hold on;
plot(hkTime, tempMcu, "Color", [0.12 0.36 0.72]);
plot(hkTime, tempBoard, "Color", [0.20 0.55 0.30]);
ylabel("Temperature (deg C)");
title("Measured satellite temperatures", "FontSize", 14);
legend("Solar panel mean", "MCU", "Board", "Location", "northoutside", "Orientation", "horizontal");
styleTimeAxis(ax, hkTime);

ax = nexttile(tlThermal);
stairs(hkTime, double(eclipseFlag), "Color", [0.10 0.10 0.10], "LineWidth", 2.0);
ylim([-0.08 1.08]);
yticks([0 1]);
yticklabels(["Sunlight", "Eclipse"]);
ylabel("Orbit state");
xlabel("UTC time");
title("Predicted eclipse periods from TLE orbit", "FontSize", 14);
styleTimeAxis(ax, hkTime);
polishFigure(figThermal);
saveReadableFigure(figThermal, figureOutDir, "01_thermal_cycle_eclipse", openExportedFigureWindows, closeFiguresAfterExport);

figComm = readableFigure("Communication quality time series", [80 60 1180 900]);
tlComm = tiledlayout(figComm, 3, 1, "TileSpacing", "loose", "Padding", "loose");
title(tlComm, "Communication quality and Zhongli geometry", "FontSize", 16, "FontWeight", "bold");
ax = nexttile(tlComm);
plot(hkTime, lastRssi, "Color", [0.13 0.37 0.76]);
ylabel("last\_rssi");
title("RSSI over time", "FontSize", 14);
styleTimeAxis(ax, hkTime);

ax = nexttile(tlComm);
plot(hkTime, elevationDeg, "Color", [0.90 0.42 0.10]);
ylabel("Elevation (deg)");
title("Ground-station elevation", "FontSize", 14);
styleTimeAxis(ax, hkTime);

ax = nexttile(tlComm);
plot(hkTime, rangeKm, "Color", [0.34 0.49 0.18]);
ylabel("Range (km)");
xlabel("UTC time");
title("Satellite range from Zhongli", "FontSize", 14);
styleTimeAxis(ax, hkTime);
polishFigure(figComm);
saveReadableFigure(figComm, figureOutDir, "02_communication_time_series", openExportedFigureWindows, closeFiguresAfterExport);

figCommScatter = readableFigure("RSSI versus elevation", [180 100 900 650]);
axes(figCommScatter);
goodScatter = isfinite(lastRssi) & isfinite(elevationDeg) & isfinite(rangeKm);
scatter(elevationDeg(goodScatter), lastRssi(goodScatter), 36, rangeKm(goodScatter), "filled", ...
    "MarkerFaceAlpha", 0.85, "MarkerEdgeColor", [1 1 1] * 0.25);
xlabel("Elevation (deg)");
ylabel("last\_rssi");
title("RSSI versus ground-station elevation", "FontSize", 16, "FontWeight", "bold");
cb = colorbar;
cb.Label.String = "Range (km)";
grid on;
box on;
polishFigure(figCommScatter);
saveReadableFigure(figCommScatter, figureOutDir, "03_rssi_versus_elevation", openExportedFigureWindows, closeFiguresAfterExport);

figAttitude = readableFigure("Attitude pointing assessment", [120 90 1180 820]);
tlAttitude = tiledlayout(figAttitude, 3, 1, "TileSpacing", "compact", "Padding", "compact");
title(tlAttitude, "ADCS pointing reconstruction from telemetry quaternion", "FontSize", 18, "FontWeight", "bold");
ax = nexttile(tlAttitude);
plot(hkTime, nadirPointingErrorDeg, "Color", [0.13 0.37 0.76]); hold on;
yline(attitudeModeThresholdDeg, "--", "Color", [0.65 0.20 0.16]);
ylabel("Error (deg)");
title("Nadir / Earth-pointing error", "FontSize", 14);
styleTimeAxis(ax, hkTime);

ax = nexttile(tlAttitude);
plot(hkTime, sunPointingErrorDeg, "Color", [0.90 0.42 0.10]); hold on;
yline(attitudeModeThresholdDeg, "--", "Color", [0.65 0.20 0.16]);
ylabel("Error (deg)");
title("Sun-pointing error during sunlight", "FontSize", 14);
styleTimeAxis(ax, hkTime);

ax = nexttile(tlAttitude);
[modeCodes, modeLabels] = modeStringsToCodes(inferredAdcsMode);
stairs(hkTime, modeCodes, "Color", [0.20 0.55 0.30], "LineWidth", 2.0);
yticks(1:numel(modeLabels));
yticklabels(compactModeLabels(modeLabels));
ylabel("Mode");
xlabel("UTC time");
title("Inferred ADCS behavior", "FontSize", 14);
styleTimeAxis(ax, hkTime);
polishFigure(figAttitude);
saveReadableFigure(figAttitude, figureOutDir, "04_attitude_pointing_assessment", openExportedFigureWindows, closeFiguresAfterExport);

analysisMatrix = table(spTemp(:), tempMcu(:), tempBoard(:), lastRssi(:), elevationDeg(:), rangeKm(:), ...
    double(sunlitFlag(:)), solarPowerW(:), battVolt(:), obcMode(:), ...
    nadirPointingErrorDeg(:), sunPointingErrorDeg(:), ...
    VariableNames=["spTemp", "tempMcu", "tempBoard", "lastRssi", "elevationDeg", ...
    "rangeKm", "sunlit", "solarPowerW", "battVolt", "obcMode", "nadirErrDeg", "sunErrDeg"]);

R = pairwiseCorrMatrix(table2array(analysisMatrix));
figCorr = readableFigure("Telemetry correlation matrix", [160 90 980 820]);
ax = axes(figCorr);
imagesc(ax, R, [-1 1]);
axis(ax, "square");
colormap(ax, turbo);
cb = colorbar(ax);
cb.Label.String = "Pearson correlation";
names = analysisMatrix.Properties.VariableNames;
xticks(ax, 1:numel(names));
yticks(ax, 1:numel(names));
xticklabels(ax, names);
yticklabels(ax, names);
xtickangle(ax, 35);
title(ax, "Telemetry and orbit-feature correlation", "FontSize", 18, "FontWeight", "bold");
set(ax, "TickLabelInterpreter", "none", "FontSize", 11);
box(ax, "on");
polishFigure(figCorr);
saveReadableFigure(figCorr, figureOutDir, "05_correlation_matrix", openExportedFigureWindows, closeFiguresAfterExport);

fprintf("\nKey correlation checks:\n");
fprintf("corr(spTemp, sunlit)       = %.3f\n", corrPair(spTemp, double(sunlitFlag)));
fprintf("corr(last_rssi, elevation) = %.3f\n", corrPair(lastRssi, elevationDeg));
fprintf("corr(last_rssi, range)     = %.3f\n", corrPair(lastRssi, rangeKm));

%% 8. Digital twin model 1: first-order thermal prediction
thermalInput = tempBoard;
if all(isnan(thermalInput))
    thermalInput = tempMcu;
end
if all(isnan(thermalInput))
    thermalInput = spTemp;
end

thermalModel = simulateThermalModel(hkTime, thermalInput, sunlitFlag, sunIncidenceProxy, obcMode);
thermalRmse = sqrt(mean((thermalModel - thermalInput).^2, "omitnan"));

figThermalModel = readableFigure("Thermal model validation", [160 80 1180 620]);
ax = axes(figThermalModel);
plot(hkTime, thermalInput, "Color", [0.13 0.37 0.76]); hold on;
plot(hkTime, thermalModel, "--", "Color", [0.86 0.25 0.16], "LineWidth", 2.0);
ylabel("Temperature (deg C)", "Color", "k");
xlabel("UTC time", "Color", "k");
title(sprintf("Thermal model validation: measured vs predicted (RMSE = %.2f deg C)", thermalRmse), ...
    "FontSize", 16, "FontWeight", "bold", "Color", "k");
lgd = legend("Measured", "Predicted", "Location", "northoutside", "Orientation", "horizontal");
lgd.TextColor = "k";
lgd.Color = "w";
lgd.EdgeColor = [0.45 0.45 0.45];
styleTimeAxis(ax, hkTime);
polishFigure(figThermalModel);
saveReadableFigure(figThermalModel, figureOutDir, "06_thermal_model_validation", openExportedFigureWindows, closeFiguresAfterExport);

figThermalResidual = readableFigure("Thermal model residual", [200 120 1180 560]);
ax = axes(figThermalResidual);
thermalResidual = thermalInput - thermalModel;
plot(hkTime, thermalResidual, "Color", [0.24 0.24 0.24]); hold on;
yline(0, "Color", [0.65 0.65 0.65], "LineStyle", "--");
xlabel("UTC time", "Color", "k");
ylabel("Residual (deg C)", "Color", "k");
title("Thermal model residual: measured minus predicted", ...
    "FontSize", 16, "FontWeight", "bold", "Color", "k");
styleTimeAxis(ax, hkTime);
polishFigure(figThermalResidual);
saveReadableFigure(figThermalResidual, figureOutDir, "07_thermal_model_residual", openExportedFigureWindows, closeFiguresAfterExport);

%% 9. Digital twin model 2: power budget and battery SOC prediction
[loadPowerW, modeNames] = estimateObcLoadPower(obcMode);
netPowerW = solarPowerW - loadPowerW;
batteryCapacityWh = 38;        % Adjustable CubeSat-class assumption.
initialSoc = 0.70;             % Adjustable assumption when true SOC is unavailable.
soc = integrateSoc(hkTime, netPowerW, batteryCapacityWh, initialSoc);

figPower = readableFigure("Power budget and battery prediction", [220 80 1180 850]);
tlPower = tiledlayout(figPower, 4, 1, "TileSpacing", "compact", "Padding", "compact");
title(tlPower, "Power budget and battery state prediction", "FontSize", 18, "FontWeight", "bold");
ax = nexttile(tlPower);
plot(hkTime, solarPowerW, "Color", [0.90 0.42 0.10]); hold on;
plot(hkTime, loadPowerW, "Color", [0.13 0.37 0.76]);
ylabel("Power (W)");
title("Solar generation and estimated OBC load", "FontSize", 14);
legend("Solar generation", "Estimated load", "Location", "northoutside", "Orientation", "horizontal");
styleTimeAxis(ax, hkTime);

ax = nexttile(tlPower);
plot(hkTime, netPowerW, "Color", [0.20 0.55 0.30]);
yline(0, "Color", [0.55 0.55 0.55], "LineStyle", "--");
ylabel("Net power (W)");
title("Positive is charging; negative is discharging", "FontSize", 14);
styleTimeAxis(ax, hkTime);

ax = nexttile(tlPower);
plot(hkTime, soc * 100, "Color", [0.48 0.22 0.68]);
ylim([0 100]);
ylabel("SOC (%)");
title("Predicted battery state of charge", "FontSize", 14);
styleTimeAxis(ax, hkTime);

ax = nexttile(tlPower);
plot(hkTime, battVolt, "Color", [0.10 0.55 0.65]);
xlabel("UTC time");
ylabel("Voltage (V)");
title("Measured battery voltage telemetry", "FontSize", 14);
styleTimeAxis(ax, hkTime);
polishFigure(figPower);
saveReadableFigure(figPower, figureOutDir, "08_power_budget_battery", openExportedFigureWindows, closeFiguresAfterExport);

modeTable = table(obcMode(:), loadPowerW(:), modeNames(:), ...
    VariableNames=["obcMode", "loadPowerW", "modeName"]);
modeSummary = groupsummary(modeTable, "obcMode", "mean", "loadPowerW");
disp("====== Estimated OBC mode power table ======");
disp(modeSummary);

%% 10. Portfolio report exports
if exportPortfolioReport
    telemetrySpanHours = hours(hkTime(end) - hkTime(1));
    summaryMetrics = table( ...
        ["telemetry_rows"; "telemetry_span"; "zhongli_access_samples"; "zhongli_access_fraction"; ...
         "quaternion_coverage"; "median_nadir_pointing_error"; "p95_nadir_pointing_error"; ...
         "median_sun_pointing_error"; "thermal_model_rmse"; "minimum_predicted_soc"; ...
         "final_predicted_soc"; "mean_solar_generation"; "mean_net_power"], ...
        [height(hk); telemetrySpanHours; nnz(accessVector); mean(accessVector); ...
         100 * mean(validQ); median(nadirPointingErrorDeg, "omitnan"); finitePercentile(nadirPointingErrorDeg, 95); ...
         median(sunPointingErrorDeg, "omitnan"); thermalRmse; 100 * min(soc, [], "omitnan"); ...
         100 * soc(end); mean(solarPowerW, "omitnan"); mean(netPowerW, "omitnan")], ...
        ["rows"; "hours"; "samples"; "fraction"; "percent"; "deg"; "deg"; ...
         "deg"; "deg C"; "percent"; "percent"; "W"; "W"], ...
        VariableNames=["Metric", "Value", "Unit"]);

    summaryCsv = fullfile(outDir, "RIoT2_portfolio_summary_metrics.csv");
    writetable(summaryMetrics, summaryCsv);

    attitudeCsv = fullfile(outDir, "RIoT2_attitude_pointing_and_mode.csv");
    attitudeTable = table(hkTime(:), validQ(:), nadirPointingErrorDeg(:), sunPointingErrorDeg(:), ...
        inferredAdcsMode(:), elevationDeg(:), rangeKm(:), sunlitFlag(:), ...
        VariableNames=["UtcTime", "ValidQuaternion", "NadirPointingErrorDeg", ...
        "SunPointingErrorDeg", "InferredAdcsMode", "ElevationDeg", "RangeKm", "Sunlit"]);
    writetable(attitudeTable, attitudeCsv);

    if any(validQ)
        modeDistribution = groupsummary(table(inferredAdcsMode(validQ), ...
            VariableNames="InferredAdcsMode"), "InferredAdcsMode");
    else
        modeDistribution = table(strings(0, 1), zeros(0, 1), ...
            VariableNames=["InferredAdcsMode", "GroupCount"]);
    end
    modeCsv = fullfile(outDir, "RIoT2_inferred_adcs_mode_distribution.csv");
    writetable(modeDistribution, modeCsv);

    reportFile = fullfile(outDir, "RIoT2_portfolio_report.md");
    writePortfolioReport(reportFile, summaryMetrics, modeDistribution, figureOutDir);

    fprintf("\nPortfolio outputs written:\n");
    fprintf("  %s\n", summaryCsv);
    fprintf("  %s\n", attitudeCsv);
    fprintf("  %s\n", modeCsv);
    fprintf("  %s\n", reportFile);
end

%% 11. Optional 3D playback
if isequal(show3DViewer, true)
    viewer = satelliteScenarioViewer(sc, "ShowDetails", true);
    show(sat);
    show(gs);
    camtarget(viewer, sat);
    play(sc);
end

%% ================= Local functions =================
function archiveDir = findFirstTelemetryArchive(searchRoots)
    archiveDir = "";
    for i = 1:numel(searchRoots)
        root = string(searchRoots(i));
        if strlength(root) == 0 || ~isfolder(root)
            continue;
        end

        directFiles = dir(fullfile(root, "*-RIoT-2-hk.csv"));
        directFiles = directFiles(~startsWith({directFiles.name}, "._"));
        if ~isempty(directFiles)
            archiveDir = root;
            return;
        end

        nestedFiles = dir(fullfile(root, "**", "*-RIoT-2-hk.csv"));
        nestedFiles = nestedFiles(~startsWith({nestedFiles.name}, "._"));
        nestedFiles = nestedFiles(~contains(string({nestedFiles.folder}), "__MACOSX"));
        if ~isempty(nestedFiles)
            archiveDir = string(nestedFiles(1).folder);
            return;
        end
    end
end

function fig = readableFigure(name, position)
    fig = figure("Name", name, ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "WindowStyle", "normal", ...
        "Units", "pixels", ...
        "Position", position);
    try
        movegui(fig, "center");
    catch
    end
end

function polishFigure(fig)
    darkText = [0.08 0.08 0.08];
    gridColor = [0.82 0.82 0.82];

    fig.Color = "w";

    axs = findall(fig, "Type", "Axes");
    for k = 1:numel(axs)
        axs(k).Color = "w";
        axs(k).XColor = darkText;
        axs(k).YColor = darkText;
        axs(k).ZColor = darkText;
        axs(k).GridColor = gridColor;
        axs(k).MinorGridColor = [0.90 0.90 0.90];
        axs(k).Title.Color = darkText;
        axs(k).XLabel.Color = darkText;
        axs(k).YLabel.Color = darkText;
        axs(k).ZLabel.Color = darkText;
        axs(k).Title.FontWeight = "bold";
        axs(k).XLabel.FontWeight = "normal";
        axs(k).YLabel.FontWeight = "normal";
    end

    legends = findall(fig, "Type", "Legend");
    for k = 1:numel(legends)
        legends(k).Color = "w";
        legends(k).TextColor = darkText;
        legends(k).EdgeColor = [0.55 0.55 0.55];
        legends(k).Box = "on";
    end

    colorbars = findall(fig, "Type", "ColorBar");
    for k = 1:numel(colorbars)
        colorbars(k).Color = darkText;
        colorbars(k).Label.Color = darkText;
    end

    labels = findall(fig, "Type", "Text");
    for k = 1:numel(labels)
        labels(k).Color = darkText;
    end

    textLike = findall(fig, "-property", "ForegroundColor");
    for k = 1:numel(textLike)
        try
            textLike(k).ForegroundColor = darkText;
        catch
        end
    end
end

function outFile = saveReadableFigure(fig, outDir, baseName, openWindow, closeAfterExport)
    if ~isfolder(outDir)
        mkdir(outDir);
    end

    polishFigure(fig);
    drawnow;
    outFile = fullfile(outDir, baseName + ".png");
    exportgraphics(fig, outFile, "Resolution", 180, "BackgroundColor", "white");

    if openWindow
        try
            winopen(outFile);
        catch
            fprintf("Saved figure: %s\n", outFile);
        end
    else
        fprintf("Saved figure: %s\n", outFile);
    end

    if closeAfterExport
        close(fig);
    end
end

function styleTimeAxis(ax, t)
    grid(ax, "on");
    box(ax, "on");
    ax.Color = "w";
    ax.GridColor = [0.82 0.82 0.82];
    ax.GridAlpha = 0.45;
    ax.MinorGridColor = [0.90 0.90 0.90];
    ax.XColor = [0.12 0.12 0.12];
    ax.YColor = [0.12 0.12 0.12];
    ax.FontSize = 12;
    ax.LineWidth = 1.0;
    if ~isempty(t)
        xlim(ax, [t(1) t(end)]);
    end
end

function hk = readAndStackTelemetry(hkFiles)
    hk = table();
    for i = 1:numel(hkFiles)
        filePath = fullfile(hkFiles(i).folder, hkFiles(i).name);
        opts = detectImportOptions(filePath, "VariableNamingRule", "preserve");
        T = readtable(filePath, opts);
        T.SourceFile = repmat(string(hkFiles(i).name), height(T), 1);
        if isempty(hk)
            hk = T;
        else
            hk = vertcat(hk, T); %#ok<AGROW>
        end
    end
end

function t = telemetryTime(T)
    if hasVar(T, "clock")
        unixSeconds = getVar(T, "clock");
    elseif hasVar(T, "uGNSSsuppliedunixtimeintegerseconds")
        unixSeconds = getVar(T, "uGNSSsuppliedunixtimeintegerseconds");
    elseif hasVar(T, "uRTCTimes")
        unixSeconds = getVar(T, "uRTCTimes");
    else
        error("No usable time column was found.");
    end
    t = datetime(unixSeconds, "ConvertFrom", "posixtime", "TimeZone", "UTC");
end

function tf = hasVar(T, names)
    tf = ismember(string(names), string(T.Properties.VariableNames));
end

function x = getVar(T, name)
    name = string(name);
    if hasVar(T, name)
        x = T.(name);
        if iscell(x) || isstring(x)
            x = str2double(string(x));
        end
        x = double(x);
    else
        x = nan(height(T), 1);
    end
end

function y = meanAvailable(X)
    y = mean(X, 2, "omitnan");
end

function y = sumAvailable(X)
    y = sum(X, 2, "omitnan");
end

function y = firstAvailable(X)
    y = nan(size(X, 1), 1);
    for c = 1:size(X, 2)
        take = isnan(y) & isfinite(X(:, c));
        y(take) = X(take, c);
    end
end

function tempC = scaleMaybeTenthsC(rawTemp)
    tempC = rawTemp;
    if median(abs(rawTemp), "omitnan") > 150
        tempC = rawTemp / 10;
    end
end

function R = eciFromLvlh(r, v)
    r = r(:);
    v = v(:);
    z = -r / norm(r);
    h = cross(r, v);
    y = -h / norm(h);
    x = cross(y, z);
    R = [x, y, z];
end

function dcm = quatScalarLastToDcm(q)
    qx = q(1); qy = q(2); qz = q(3); qw = q(4);
    dcm = [1 - 2*(qy^2 + qz^2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy); ...
           2*(qx*qy + qw*qz), 1 - 2*(qx^2 + qz^2), 2*(qy*qz - qw*qx); ...
           2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx^2 + qy^2)];
end

function angleDeg = angularSeparationDeg(a, b)
    if any(~isfinite(a)) || any(~isfinite(b)) || norm(a) == 0 || norm(b) == 0
        angleDeg = nan;
        return;
    end
    c = dot(a(:), b(:)) / (norm(a) * norm(b));
    c = min(1, max(-1, c));
    angleDeg = acosd(c);
end

function modeName = classifyAttitudeMode(nadirErrorDeg, sunErrorDeg, sunlitFlag, thresholdDeg)
    if isfinite(nadirErrorDeg) && nadirErrorDeg <= thresholdDeg
        modeName = "nadir_pointing";
    elseif sunlitFlag && isfinite(sunErrorDeg) && sunErrorDeg <= thresholdDeg
        modeName = "sun_pointing";
    elseif isfinite(nadirErrorDeg) || isfinite(sunErrorDeg)
        modeName = "slew_or_unclassified";
    else
        modeName = "no_quaternion";
    end
end

function [codes, labels] = modeStringsToCodes(modeStrings)
    labels = unique(modeStrings, "stable");
    codes = nan(size(modeStrings));
    for i = 1:numel(labels)
        codes(modeStrings == labels(i)) = i;
    end
end

function labelsOut = compactModeLabels(labelsIn)
    labelsOut = replace(labelsIn, "_", " ");
    labelsOut(labelsIn == "nadir_pointing") = "nadir";
    labelsOut(labelsIn == "sun_pointing") = "sun";
    labelsOut(labelsIn == "slew_or_unclassified") = "slew/unclass.";
    labelsOut(labelsIn == "no_quaternion") = "no q";
end

function sunHat = approximateSunEciUnit(t)
    jd = juliandate(t);
    n = jd - 2451545.0;
    L = mod(280.460 + 0.9856474 * n, 360);
    g = deg2rad(mod(357.528 + 0.9856003 * n, 360));
    lambda = deg2rad(L + 1.915 * sin(g) + 0.020 * sin(2 * g));
    epsilon = deg2rad(23.439 - 0.0000004 * n);
    sunHat = [cos(lambda); cos(epsilon) * sin(lambda); sin(epsilon) * sin(lambda)];
    sunHat = sunHat / norm(sunHat);
end

function tf = isInCylindricalEclipse(rEciMeters, sunHat)
    earthRadiusMeters = 6378.137e3;
    behindEarth = dot(rEciMeters, sunHat) < 0;
    distanceFromSunLine = norm(cross(rEciMeters, sunHat));
    tf = behindEarth && distanceFromSunLine < earthRadiusMeters;
end

function r = corrPair(a, b)
    good = isfinite(a) & isfinite(b);
    if nnz(good) < 3
        r = nan;
    else
        C = corrcoef(a(good), b(good));
        r = C(1, 2);
    end
end

function R = pairwiseCorrMatrix(X)
    nVars = size(X, 2);
    R = nan(nVars);
    for i = 1:nVars
        for j = 1:nVars
            good = isfinite(X(:, i)) & isfinite(X(:, j));
            if nnz(good) >= 3
                C = corrcoef(X(good, i), X(good, j));
                R(i, j) = C(1, 2);
            end
        end
    end
end

function Tpred = simulateThermalModel(t, Tmeas, sunlitFlag, incidence, obcMode)
    Tpred = nan(size(Tmeas));
    first = find(isfinite(Tmeas), 1, "first");
    if isempty(first)
        return;
    end
    Tpred(first) = Tmeas(first);

    tauSeconds = 28 * 60;
    coldEq = -12;
    hotEqBase = 42;
    modeHeat = 0.6 * fillmissing(obcMode, "constant", median(obcMode, "omitnan"));
    modeHeat = modeHeat - median(modeHeat, "omitnan");

    for k = first+1:numel(Tpred)
        dt = seconds(t(k) - t(k-1));
        if ~isfinite(dt) || dt <= 0
            dt = 60;
        end
        solarGain = 22 * incidence(k) + 10 * double(sunlitFlag(k));
        equilibrium = coldEq + solarGain + modeHeat(k);
        if sunlitFlag(k)
            equilibrium = max(equilibrium, hotEqBase + 8 * incidence(k) + modeHeat(k));
        end
        alpha = 1 - exp(-dt / tauSeconds);
        Tpred(k) = Tpred(k-1) + alpha * (equilibrium - Tpred(k-1));
    end
end

function [loadPowerW, modeNames] = estimateObcLoadPower(obcMode)
    loadPowerW = nan(size(obcMode));
    modeNames = strings(size(obcMode));
    for i = 1:numel(obcMode)
        switch obcMode(i)
            case 0
                loadPowerW(i) = 2.2; modeNames(i) = "boot";
            case 1
                loadPowerW(i) = 3.0; modeNames(i) = "safe";
            case 2
                loadPowerW(i) = 3.8; modeNames(i) = "standby";
            case 3
                loadPowerW(i) = 4.5; modeNames(i) = "mission";
            case 4
                loadPowerW(i) = 5.2; modeNames(i) = "contact";
            case 5
                loadPowerW(i) = 6.0; modeNames(i) = "high-rate";
            otherwise
                loadPowerW(i) = 4.0 + 0.25 * max(0, obcMode(i));
                modeNames(i) = "custom";
        end
    end
    loadPowerW = fillmissing(loadPowerW, "constant", 4.0);
end

function soc = integrateSoc(t, netPowerW, capacityWh, initialSoc)
    soc = nan(size(netPowerW));
    soc(1) = initialSoc;
    netPowerW = fillmissing(netPowerW, "constant", 0);
    for k = 2:numel(soc)
        dtHours = hours(t(k) - t(k-1));
        if ~isfinite(dtHours) || dtHours < 0
            dtHours = 0;
        end
        soc(k) = min(1, max(0, soc(k-1) + netPowerW(k) * dtHours / capacityWh));
    end
end

function p = finitePercentile(x, pct)
    x = sort(x(isfinite(x)));
    if isempty(x)
        p = nan;
        return;
    end
    pos = 1 + (numel(x) - 1) * pct / 100;
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        p = x(lo);
    else
        p = x(lo) + (x(hi) - x(lo)) * (pos - lo);
    end
end

function writePortfolioReport(reportFile, summaryMetrics, modeDistribution, figureOutDir)
    lines = strings(0, 1);
    lines(end+1) = "# RIoT-2 Orbit, Telemetry, and ADCS Digital Twin Report";
    lines(end+1) = "";
    lines(end+1) = "## Executive Summary";
    lines(end+1) = sprintf("- Telemetry rows analyzed: %.0f", metricValue(summaryMetrics, "telemetry_rows"));
    lines(end+1) = sprintf("- Telemetry span: %.2f hours", metricValue(summaryMetrics, "telemetry_span"));
    lines(end+1) = sprintf("- Zhongli access fraction: %.2f %%", 100 * metricValue(summaryMetrics, "zhongli_access_fraction"));
    lines(end+1) = sprintf("- Quaternion coverage: %.2f %%", metricValue(summaryMetrics, "quaternion_coverage"));
    lines(end+1) = sprintf("- Median nadir pointing error: %.2f deg", metricValue(summaryMetrics, "median_nadir_pointing_error"));
    lines(end+1) = sprintf("- Median sun pointing error: %.2f deg", metricValue(summaryMetrics, "median_sun_pointing_error"));
    lines(end+1) = sprintf("- Thermal digital-twin RMSE: %.2f deg C", metricValue(summaryMetrics, "thermal_model_rmse"));
    lines(end+1) = sprintf("- Minimum predicted SOC: %.2f %%", metricValue(summaryMetrics, "minimum_predicted_soc"));
    lines(end+1) = "";
    lines(end+1) = "## Inferred ADCS Mode Distribution";
    if isempty(modeDistribution)
        lines(end+1) = "- No valid quaternion samples were available for mode inference.";
    else
        for i = 1:height(modeDistribution)
            lines(end+1) = sprintf("- %s: %.0f samples", ...
                string(modeDistribution.InferredAdcsMode(i)), modeDistribution.GroupCount(i));
        end
    end
    lines(end+1) = "";
    lines(end+1) = "## Generated Figures";
    figureFiles = dir(fullfile(figureOutDir, "*.png"));
    for i = 1:numel(figureFiles)
        lines(end+1) = "- " + string(figureFiles(i).name);
    end
    lines(end+1) = "";
    lines(end+1) = "## Portfolio Notes";
    lines(end+1) = "- The analysis combines TLE orbit propagation, ground-station geometry, telemetry correlation, quaternion-based ADCS reconstruction, and first-order thermal/power digital-twin models.";
    lines(end+1) = "- ADCS mode inference is based on configurable body-axis assumptions and pointing-error thresholds, so it should be calibrated with spacecraft mechanical documentation when available.";

    writelines(lines, reportFile);
end

function v = metricValue(summaryMetrics, metricName)
    idx = summaryMetrics.Metric == string(metricName);
    if any(idx)
        v = summaryMetrics.Value(find(idx, 1, "first"));
    else
        v = nan;
    end
end

