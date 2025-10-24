# AGPL-3.0 Lisans Rehberi

## Nedir?

Jessy projesi **GNU Affero General Public License v3.0 (AGPL-3.0)** ile lisanslanmıştır.

## Ne Yapabilirsiniz?

### ✅ İzin Verilenler

1. **Kullanabilirsiniz** - Ücretsiz olarak kullanın
2. **Değiştirebilirsiniz** - Kodu istediğiniz gibi değiştirin
3. **Dağıtabilirsiniz** - Başkalarıyla paylaşın
4. **Ticari kullanabilirsiniz** - Para kazanmak için kullanın
5. **Patent koruması** - Patent haklarından korunursunuz

### ⚠️ Şartlar

1. **Kaynak kodu paylaşmalısınız** - Değiştirdiğiniz kodu açık kaynak yapmalısınız
2. **Aynı lisansı kullanmalısınız** - Türev çalışmalar da AGPL-3.0 olmalı
3. **Değişiklikleri belirtmelisiniz** - Ne değiştirdiğinizi yazmalısınız
4. **Lisans metnini dahil edin** - LICENSE dosyasını kopyalayın
5. **Network kullanımı = dağıtım** - Web servisi olarak bile çalıştırsanız, kaynak kodu paylaşmalısınız

## AGPL-3.0 vs Diğer Lisanslar

### MIT Lisansı ile Fark
- **MIT**: Herkes kodu alıp kapalı kaynak yapabilir
- **AGPL-3.0**: Herkes kodu kullanabilir AMA değişiklikleri paylaşmak ZORUNDA

### GPL-3.0 ile Fark
- **GPL-3.0**: Sadece yazılımı dağıtırsanız kaynak kodu paylaşmalısınız
- **AGPL-3.0**: Web servisi olarak bile çalıştırsanız kaynak kodu paylaşmalısınız

## Örnekler

### ✅ İzin Verilen Kullanım

```bash
# Kendi projenizde kullanın
git clone https://github.com/gokselozgur5/jessy.git
cd jessy
# Kullanın, değiştirin, geliştirin

# Değişikliklerinizi paylaşın
git add .
git commit -m "feat: yeni özellik eklendi"
git push origin main
```

### ⚠️ Şartlı Kullanım

**Senaryo 1: Web Servisi Olarak Çalıştırma**
```bash
# Jessy'yi web servisi olarak çalıştırıyorsunuz
docker-compose up -d

# ⚠️ ZORUNLU: Kaynak kodunuzu paylaşmalısınız
# Kullanıcılarınıza kaynak koda erişim sağlamalısınız
```

**Senaryo 2: Değiştirip Dağıtma**
```bash
# Jessy'yi değiştirdiniz
vim src/lib.rs

# ⚠️ ZORUNLU: Değişikliklerinizi AGPL-3.0 ile paylaşmalısınız
git add .
git commit -m "feat: özel özellik"
git push # Açık kaynak olarak
```

### ❌ İzin Verilmeyen Kullanım

1. **Kapalı kaynak yapma**
   ```
   ❌ Jessy'yi alıp, değiştirip, kapalı kaynak ürün yapma
   ❌ Kaynak kodu gizleyip satma
   ❌ Değişiklikleri paylaşmadan web servisi çalıştırma
   ```

2. **Lisans değiştirme**
   ```
   ❌ AGPL-3.0'ı MIT'ye çevirme
   ❌ Proprietary lisans ekleme
   ❌ Lisans metnini kaldırma
   ```

## Neden AGPL-3.0?

### Proje Sahibinin Amacı

> "Ben açıyım kullanılsın ama kimse benden habersiz bir şey yapamaz istiyorum"

AGPL-3.0 tam olarak bunu sağlar:

1. ✅ **Açık kaynak** - Herkes kullanabilir
2. ✅ **Koruma** - Kimse kapalı kaynak yapamaz
3. ✅ **Şeffaflık** - Tüm değişiklikler görünür
4. ✅ **Topluluk** - İyileştirmeler herkese fayda sağlar

### Copyleft Koruması

AGPL-3.0 "copyleft" lisansıdır:
- Özgürlükleri korur
- Kapalı kaynak olmayı engeller
- Topluluk faydasını garanti eder

## Ticari Kullanım

### ✅ İzin Verilen

```bash
# Jessy'yi kullanarak para kazanabilirsiniz
# Örnek: Danışmanlık hizmeti
# Örnek: Hosting hizmeti
# Örnek: Destek hizmeti

# ŞART: Kaynak kodunuzu paylaşın
```

### ❌ İzin Verilmeyen

```bash
# Jessy'yi alıp kapalı kaynak SaaS yapma
# Kaynak kodu gizleyip satma
# Proprietary ürün olarak pazarlama
```

## Katkıda Bulunma

Jessy'ye katkıda bulunmak istiyorsanız:

1. Fork yapın
2. Değişikliklerinizi yapın
3. Pull request gönderin
4. Katkınız AGPL-3.0 ile lisanslanır

```bash
git clone https://github.com/YOUR_USERNAME/jessy.git
cd jessy
git checkout -b feature/yeni-ozellik

# Değişikliklerinizi yapın
git add .
git commit -m "feat: yeni özellik"
git push origin feature/yeni-ozellik

# Pull request oluşturun
```

## SSS (Sık Sorulan Sorular)

### S: Jessy'yi kendi projemde kullanabilir miyim?
**C:** Evet! Ama projeniz de AGPL-3.0 olmalı.

### S: Jessy'yi web servisi olarak çalıştırabilir miyim?
**C:** Evet! Ama kaynak kodunuzu kullanıcılarınıza sunmalısınız.

### S: Jessy'yi değiştirip satabilir miyim?
**C:** Evet! Ama değişikliklerinizi AGPL-3.0 ile paylaşmalısınız.

### S: Jessy'yi kapalı kaynak yapabilir miyim?
**C:** Hayır! Bu AGPL-3.0 ihlalidir.

### S: Başka bir lisans alabilir miyim?
**C:** Proje sahibiyle iletişime geçin. Dual-licensing mümkün olabilir.

## Lisans İhlali

AGPL-3.0'ı ihlal ederseniz:

1. ⚠️ Lisans haklarınızı kaybedersiniz
2. ⚠️ Yasal işlem başlatılabilir
3. ⚠️ Telif hakkı ihlali suçu işlemiş olursunuz

## Daha Fazla Bilgi

- **Tam lisans metni**: [LICENSE](../LICENSE)
- **AGPL-3.0 resmi sayfa**: https://www.gnu.org/licenses/agpl-3.0.html
- **AGPL-3.0 SSS**: https://www.gnu.org/licenses/gpl-faq.html
- **Copyleft nedir**: https://www.gnu.org/licenses/copyleft.html

## İletişim

Lisans hakkında sorularınız için:
- GitHub Issues: https://github.com/gokselozgur5/jessy/issues
- Email: gokselozgur5@gmail.com

---

*"Özgür yazılım, özgür toplum. AGPL-3.0 ile korunuyor. 🎪"*
