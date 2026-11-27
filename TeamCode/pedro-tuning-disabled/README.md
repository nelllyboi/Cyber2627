# Pedro AutoTune (disabled)

These files are parked here, outside `src/`, so they don't compile while the
`com.pedropathing:tuning` dependency is off. That dependency pulls in Sloth, which
stops FTC Dashboard Configuration edits from reaching OpModes (needed for flywheel tuning).

The tuned Pedro values are still in `src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`.

To turn AutoTune back on:
1. Move `Tuning.java` and `procedures/` back into
   `src/main/java/org/firstinspires/ftc/teamcode/pedro/`.
2. Uncomment `implementation 'com.pedropathing:tuning:1.0.3'` in `build.dependencies.gradle`.
3. Gradle sync, then build.
