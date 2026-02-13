find src lib include -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.c" -o -name "*.hpp" \) | while IFS= read -r file; do
  rsync -av --mkpath --delete --exclude .direnv/ --exclude .git/ "$file" robo@roboberry.local:robocup2026-esp32-program/$file
done
