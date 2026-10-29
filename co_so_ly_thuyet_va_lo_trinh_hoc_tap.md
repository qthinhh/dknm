# Cơ sở lý thuyết và lộ trình học tập

## Điều khiển hình thái dựa trên sự đồng thuận cho hệ đa tác tử không đồng nhất

Tài liệu này giúp người học đi từ kiến thức nền tảng đến khả năng tự đọc, mô phỏng, phân tích và viết báo cáo về bài báo:

> Chang, X.; Yang, Y.; Zhang, Z.; Jiao, J.; Cheng, H.; Fu, W.  
> *Consensus-Based Formation Control for Heterogeneous Multi-Agent Systems in Complex Environments*.  
> Drones, 2025, 9, 175.

Tài liệu liên quan:

- [Báo cáo nghiên cứu](./bao_cao_nghien_cuu.md)
- [Mã MATLAB mô phỏng](./simulate_consensus_formation.m)
- [Hướng dẫn mô phỏng](./huong_dan_mo_phong.md)
- [Bài báo gốc](./drones-09-00175-v3.pdf)

Mục tiêu là hiểu được:

1. hệ đa tác tử được mô hình hóa như thế nào;
2. thông tin giữa các tác tử được biểu diễn bằng đồ thị ra sao;
3. vì sao luật đồng thuận làm các tác tử phối hợp được;
4. làm thế nào duy trì hình thái và tránh va chạm;
5. vì sao hệ ổn định;
6. cách chuyển lý thuyết thành mô phỏng và báo cáo khoa học.

---

## 1. Bức tranh tổng quát của bài toán

Hãy hình dung một đội UAV cùng bay trong thành phố:

- mỗi UAV có vị trí khác nhau;
- một số UAV có mô hình chuyển động đơn giản;
- một số UAV có thêm trạng thái vận tốc và gia tốc;
- UAV chỉ trao đổi thông tin với một số lân cận;
- đội hình phải tránh nhà cao tầng, vật cản và các UAV khác;
- khi đi qua khe hẹp, đội hình cần đổi từ dạng rộng sang dạng đường thẳng;
- khi ra khỏi khe hẹp, đội hình có thể mở rộng trở lại.

Đây là một hệ đa tác tử không đồng nhất.

Bài toán điều khiển cần trả lời:

> Làm thế nào để mỗi tác tử chỉ dùng thông tin cục bộ nhưng toàn bộ đội vẫn duy trì hình thái, tránh va chạm, giữ liên lạc và ổn định khi môi trường thay đổi?

Bài báo kết hợp:

```
Mô hình động lực học
        +
Lý thuyết đồ thị
        +
Đồng thuận đa tác tử
        +
Điều khiển hình thái
        +
Hàm thế tránh va chạm
        +
Phân tích Lyapunov
        +
Mô phỏng số
```

---

## 2. Kiến thức nền tảng cần có

### 2.1. Đại số tuyến tính

Cần hiểu:

- vector và ma trận;
- tích vô hướng;
- ma trận đối xứng;
- trị riêng và vector riêng;
- ma trận xác định dương hoặc bán xác định dương;
- chuẩn Euclid;
- tích Kronecker ở mức cơ bản.

Vị trí của UAV trong mặt phẳng:

\[
x_i =
\begin{bmatrix}
x_i^x\\
x_i^y
\end{bmatrix}
\in \mathbb{R}^2.
\]

Vị trí toàn đội có thể ghép thành:

\[
x =
\begin{bmatrix}
x_1\\
x_2\\
\vdots\\
x_N
\end{bmatrix}.
\]

#### Bài tập

Cho:

\[
A =
\begin{bmatrix}
0&1&0\\
1&0&1\\
0&1&0
\end{bmatrix}.
\]

Hãy tính ma trận bậc D, ma trận Laplace L=D-A và trị riêng của L. Kiểm tra:

\[
L
\begin{bmatrix}
1\\1\\1
\end{bmatrix}
=0.
\]

Nếu chưa làm được, hãy ôn đại số tuyến tính trước khi đọc phần lý thuyết đồ thị.

### 2.2. Phương trình vi phân và hệ động lực

Tác tử bậc một:

\[
\dot{x}_i=u_i.
\]

Tác tử bậc hai:

\[
\dot{x}_i=v_i,\qquad \dot{v}_i=u_i.
\]

Tác tử có nhiễu:

\[
\dot{x}_i=u_i+f_i(x_i)+d_i(t),
\]

hoặc:

\[
\dot{x}_i=v_i,\qquad
\dot{v}_i=u_i+d_i(t).
\]

Trong đó d_i(t) có thể biểu diễn gió, sai số cảm biến hoặc tác động bên ngoài.

Cần hiểu:

- trạng thái là gì;
- đầu vào điều khiển tác động lên trạng thái nào;
- tác tử bậc một khác tác tử bậc hai ở đâu;
- vì sao một luật điều khiển cho tác tử bậc một chưa chắc áp dụng trực tiếp cho tác tử bậc hai.

### 2.3. MATLAB hoặc GNU Octave

Cần biết:

- tạo vector và ma trận;
- vòng lặp for và điều kiện if;
- viết hàm;
- vẽ bằng plot;
- lưu dữ liệu bằng save;
- tính chuẩn bằng norm;
- tính trung bình bằng mean.

Ví dụ tích phân Euler:

```
x = [1; 2];
u = [0.5; -0.2];
dt = 0.01;
x_next = x + dt*u;
```

Tương ứng:

\[
x(t+\Delta t)
\approx x(t)+\Delta t\,\dot{x}(t).
\]

---

## 3. Lý thuyết đồ thị

### 3.1. Đồ thị truyền thông

Một đồ thị viết là:

\[
G=(V,E,A).
\]

- V: tập nút, mỗi nút là một tác tử;
- E: tập cạnh, biểu diễn liên kết;
- A: ma trận kề hoặc ma trận trọng số.

Nếu tác tử i nhận thông tin từ tác tử j:

\[
a_{ij}>0.
\]

Nếu không có kết nối:

\[
a_{ij}=0.
\]

Trong bài báo, trọng số giảm theo khoảng cách. Khi khoảng cách vượt bán kính truyền thông R, liên kết mất.

### 3.2. Ma trận bậc và Laplace

\[
D=\operatorname{diag}(d_1,\ldots,d_N),
\qquad d_i=\sum_j a_{ij},
\]

\[
L=D-A.
\]

Với đồ thị liên thông:

\[
L\mathbf{1}=0.
\]

Trực giác:

- Lx bằng 0 khi trạng thái đạt đồng thuận;
- trị riêng 0 liên quan đến trạng thái đồng thuận;
- các trị riêng khác phản ánh mức độ liên thông.

Trong mã mô phỏng, hàm communicationWeight tạo trọng số bằng 1 khi tác tử gần, giảm trơn trong vùng biên và bằng 0 khi mất kết nối.

### Bài tập

1. Tạo 5 tác tử trên một đường thẳng.
2. Nối hai tác tử nếu khoảng cách nhỏ hơn R.
3. Vẽ ma trận A.
4. Tính D và L.
5. Thay đổi R và quan sát số liên kết.

---

## 4. Lý thuyết đồng thuận

### 4.1. Đồng thuận cơ bản

Với tác tử bậc một:

\[
u_i=-k\sum_{j\in\mathcal{N}_i}a_{ij}(x_i-x_j).
\]

Dạng ma trận:

\[
\dot{x}=-kLx.
\]

Khi đồ thị liên thông và điều kiện phù hợp được thỏa mãn:

\[
\lim_{t\to\infty}(x_i-x_j)=0.
\]

Mỗi tác tử chỉ cần dùng sai khác với các lân cận, nhưng thông tin vẫn lan truyền qua toàn mạng.

### 4.2. Đồng thuận có leader

Với leader:

\[
u_i=
-k\sum_j a_{ij}(x_i-x_j)
-k_i^L(x_i-x_L).
\]

Ma trận:

\[
H=L+B,
\]

trong đó B biểu diễn liên kết với leader.

### 4.3. Đồng thuận đa nhóm

Đồng thuận thông thường đưa tất cả tác tử về cùng một trạng thái. Bài báo cần nhiều nhóm có mục tiêu khác nhau nên đưa vào hệ số mức độ thông minh omega_i.

Ví dụ:

- omega=1.0: nhóm 1;
- omega=1.1: nhóm 2;
- omega=1.2: nhóm 3.

Các tác tử cùng nhóm được khuyến khích đạt cùng quan hệ hình thái, còn các nhóm khác nhau có thể có mục tiêu khác nhau.

Trong mã:

```
omegaValues = linspace(1.0, 1.2, nGroups);
omega = omegaValues(group).';
```

Cần nhớ rằng omega_i ở đây là tham số phân nhóm trong công trình, không phải một đại lượng chuẩn duy nhất cho mọi hệ đa tác tử.

---

## 5. Điều khiển hình thái

### 5.1. Hình thái là gì?

Gọi rho_i là vị trí tương đối mong muốn của tác tử i trong hệ quy chiếu leader:

\[
x_i^d=x_L+R(\theta)\rho_i.
\]

Trong đó:

- x_L: vị trí leader;
- rho_i: offset hình thái;
- R(theta): ma trận quay;
- x_i^d: vị trí mong muốn.

Sai số:

\[
e_i=x_i^d-x_i.
\]

### 5.2. Luật bám hình thái

Một luật đơn giản:

\[
u_i =
v_L+k_p e_i+
k_c\sum_{j\in\mathcal{N}_i}
\left[
(x_j-x_i)-R(\theta)(\rho_j-\rho_i)
\right].
\]

Các thành phần:

1. v_L: giúp đội hình đi theo leader;
2. k_p e_i: kéo tác tử về vị trí hình thái;
3. thành phần tổng lân cận: sửa sai khác tương đối.

Trong mã:

- eTarget là sai số tới vị trí đích;
- kFormation là hệ số bám hình thái;
- desiredRelative là vị trí tương đối mong muốn;
- kConsensus là hệ số đồng thuận.

### 5.3. Đổi hình thái

Mô phỏng sử dụng:

- 0–8 s: hình tròn;
- 8–16 s: đường thẳng;
- 16–24 s: tam giác.

Khi rho_i thay đổi, vị trí đích thay đổi. Đây là một hệ chuyển mạch vì luật điều khiển trước và sau thời điểm đổi hình thái khác nhau.

#### Bài tập

Thay đổi hàm desiredOffsets để tạo:

- hình chữ nhật;
- hình chữ V;
- hai hàng song song;
- hình vòng cung.

Sau đó kiểm tra khoảng cách, va chạm và sai số khi đổi hình thái.

---

## 6. Hàm thế và tránh va chạm

### 6.1. Ý tưởng hàm thế

Hàm thế gán năng lượng cho trạng thái:

- gần va chạm: năng lượng tăng;
- sai hình thái: năng lượng tăng;
- mất liên kết: năng lượng tăng;
- đạt mục tiêu: năng lượng giảm.

Điều khiển lấy từ gradient:

\[
u_i=-\nabla_{x_i}V.
\]

### 6.2. Tránh tác tử khác

Gọi:

\[
d_{ij}=\|x_i-x_j\|.
\]

Nếu d_ij nhỏ hơn khoảng cách an toàn, tác tử nhận lực đẩy theo hướng x_i-x_j.

Trong mã, các thành phần tương ứng là delta, direction và repulsion.

### 6.3. Duy trì kết nối

Khoảng cách hình thái cần nằm trong vùng cho phép:

\[
r_{\mathrm{out}}
\leq
\|\rho_{ij}\|
\leq R.
\]

R là bán kính truyền thông. Hàm thế kết nối tạo xu hướng kéo các tác tử về vùng khoảng cách phù hợp.

### 6.4. Giới hạn vận tốc

Thành phần hãm:

\[
u_v=-\beta v_i.
\]

Mục tiêu:

- giảm dao động;
- giảm năng lượng động học;
- tránh lệnh quá lớn;
- hỗ trợ chứng minh đạo hàm Lyapunov không dương.

### 6.5. Hạn chế

Hàm thế có thể gặp:

- cực tiểu cục bộ;
- kẹt giữa nhiều vật cản;
- lực quá lớn khi tác tử đi vào vật cản;
- khó xử lý vật cản chuyển động nhanh;
- phụ thuộc vào bước thời gian mô phỏng.

---

## 7. Ổn định Lyapunov và LaSalle

### 7.1. Hàm Lyapunov

Chọn hàm giống năng lượng V(x):

\[
V(x)\geq 0,
\qquad
\dot V(x)\leq 0.
\]

Nếu V không tăng, hệ không rời xa vô hạn khỏi miền ban đầu.

### 7.2. Kết quả trong bài báo

Hàm Lyapunov kết hợp:

- thế năng tránh va chạm;
- thế năng duy trì kết nối;
- sai số đồng thuận;
- năng lượng vận tốc.

Sau khi thay luật điều khiển:

\[
\dot V=-\beta \hat v^T\hat v\leq0.
\]

Thành phần hãm vận tốc làm giảm năng lượng hệ.

### 7.3. Vai trò của LaSalle

Từ dot V nhỏ hơn hoặc bằng 0, ta biết V không tăng nhưng chưa chắc mọi sai số đã bằng 0. Nguyên lý bất biến LaSalle được dùng để:

1. xác định tập dot V bằng 0;
2. tìm tập bất biến lớn nhất;
3. chứng minh quỹ đạo tiến về tập đó;
4. kết luận vận tốc và sai số hội tụ.

### 7.4. Phân biệt các loại hội tụ

- ổn định tiệm cận: hội tụ khi t tiến tới vô cùng;
- ổn định thời gian hữu hạn: hội tụ trong thời gian hữu hạn;
- ổn định thời gian đặt trước: thời gian hội tụ được chỉ định trước.

Phần chứng minh chính của bài báo dựa trên hội tụ tiệm cận và LaSalle. Do đó cần thận trọng khi diễn giải thành hội tụ thời gian đặt trước nếu không có chặn thời gian rõ ràng.

---

## 8. Đồ thị thay đổi và thời gian lưu tối thiểu

Khi tác tử di chuyển, khoảng cách thay đổi nên các cạnh truyền thông có thể xuất hiện hoặc mất đi.

Nếu các thời điểm chuyển mạng là t_k và t_{k+1}, thời gian lưu:

\[
\tau(t_k)=t_{k+1}-t_k.
\]

Điều kiện:

\[
\tau(t_k)\geq\tau_d>0.
\]

Ý nghĩa: mỗi cấu hình mạng tồn tại đủ lâu để hệ có thời gian tiến gần trạng thái ổn định trước khi chuyển sang cấu hình mới.

Nếu topology thay đổi quá nhanh, các tác tử có thể không kịp phối hợp.

---

## 9. Lộ trình học tập theo từng giai đoạn

### Giai đoạn 0 — Đọc định hướng

Đọc:

1. yeu_cau.txt;
2. Abstract và Introduction;
3. phần đầu của báo cáo nghiên cứu.

Cần trả lời:

- bài toán là gì;
- vì sao quan trọng;
- thế nào là hệ không đồng nhất;
- mục tiêu gồm đồng thuận, hình thái, tránh va chạm và giữ kết nối.

Không cần đọc toàn bộ phương trình ngay.

### Giai đoạn 1 — Đại số tuyến tính

Học vector, ma trận, trị riêng và ma trận xác định dương.

Thực hành:

1. tính D và L cho đồ thị 3 nút;
2. dùng eig(L);
3. kiểm tra L nhân vector toàn 1 bằng 0.

Đầu ra: hiểu vai trò của ma trận Laplace và trị riêng 0.

### Giai đoạn 2 — Hệ động lực

Học phương trình trạng thái, tác tử bậc một/bậc hai, Euler và nhiễu.

Thực hành:

1. một tác tử bậc một đi tới điểm đích;
2. một tác tử bậc hai đi tới điểm đích;
3. thêm giới hạn vận tốc;
4. thêm nhiễu nhỏ.

Đầu ra: biết biến phương trình vi phân thành vòng lặp mô phỏng.

### Giai đoạn 3 — Hai tác tử đồng thuận

Mô phỏng:

\[
\dot{x}_1=-k(x_1-x_2),
\qquad
\dot{x}_2=-k(x_2-x_1).
\]

Quan sát ảnh hưởng của k, bước thời gian và nhiễu.

Đầu ra: thấy trực tiếp hai tác tử hội tụ về cùng giá trị.

### Giai đoạn 4 — Nhiều tác tử và đồ thị

Thực hiện:

1. tạo 5–10 tác tử;
2. xây dựng A, D, L;
3. chạy x_dot=-kLx;
4. thay đổi topology;
5. quan sát các nhóm đồng thuận khi đồ thị không liên thông.

Đầu ra: hiểu vai trò của kết nối.

### Giai đoạn 5 — Hình thái

Tạo offset rho_i cho:

- đường thẳng;
- vòng tròn;
- tam giác.

Cho leader chuyển động và quay đội hình theo hướng leader.

Đầu ra: phân biệt consensus với formation control.

### Giai đoạn 6 — Không đồng nhất

Chia hệ thành tác tử bậc một, bậc hai và các nhóm omega khác nhau. So sánh sai số và vận tốc từng nhóm.

Đầu ra: hiểu tại sao cùng lệnh nhưng các tác tử phản ứng khác nhau.

### Giai đoạn 7 — Tránh va chạm

Thực hiện lần lượt:

1. tránh hai tác tử;
2. tránh toàn đội;
3. thêm vật cản tròn;
4. thêm vật cản chữ nhật;
5. thêm vật cản chuyển động;
6. đo khoảng cách nhỏ nhất.

Đầu ra: hiểu quan hệ giữa tránh vật cản và sai số hình thái.

### Giai đoạn 8 — Ổn định

Học hàm năng lượng, đạo hàm Lyapunov, ổn định tiệm cận, LaSalle, hệ chuyển mạch và dwell time.

Khi đọc bài báo, trả lời:

- V gồm thành phần nào;
- vì sao V không âm;
- vì sao dot V không dương;
- tập dot V bằng 0 là gì;
- LaSalle được dùng ở đâu;
- điều kiện nào cần cho topology thay đổi.

### Giai đoạn 9 — Tái hiện mô phỏng

Đọc [huong_dan_mo_phong.md](./huong_dan_mo_phong.md), chạy:

```
results = simulate_consensus_formation();
```

Quan sát sai số, tỷ lệ kết nối, đội hình ở 0, 8, 16 và 24 giây, cũng như quỹ đạo khi gặp vật cản.

---

## 10. Cách đi từ bài báo đến báo cáo

### Bước 1 — Xác định câu hỏi nghiên cứu

Viết được:

> Công trình nghiên cứu điều khiển hình thái phân tán cho hệ đa tác tử không đồng nhất trong môi trường phức tạp, trong đó các tác tử cần duy trì đội hình, tránh vật cản và thích nghi với topology truyền thông thay đổi.

### Bước 2 — Tách mô hình

Ghi riêng:

- loại tác tử;
- trạng thái;
- đầu vào;
- leader;
- followers;
- nhiễu;
- đồ thị truyền thông.

### Bước 3 — Tách luật điều khiển

Đọc thành các thành phần:

```
bám vị trí hình thái
+
đồng thuận với lân cận
+
tránh tác tử khác
+
tránh vật cản
+
hãm vận tốc
```

Giải thích vai trò từng thành phần bằng lời đơn giản.

### Bước 4 — Tách phần chứng minh

| Thành phần | Câu hỏi |
|---|---|
| Hàm Lyapunov | Đo năng lượng nào? |
| Tính không âm | Vì sao V không âm? |
| Đạo hàm | Vì sao V không tăng? |
| LaSalle | Trạng thái giới hạn là gì? |
| Dwell time | Vì sao cần thời gian lưu tối thiểu? |

### Bước 5 — Tách kết quả mô phỏng

Ghi số tác tử, loại động lực học, số nhóm, hình thái, vật cản, thời gian, tham số và kết luận quan sát được.

### Bước 6 — Phân biệt kết quả và tuyên bố

- “Hình 7 cho thấy đội hình đổi từ tròn sang đường thẳng” là kết quả quan sát được.
- “Phương pháp chống jamming” là tuyên bố cần thí nghiệm jamming cụ thể.

### Bước 7 — Viết đóng góp

Trả lời:

- tác giả kết hợp kỹ thuật nào;
- điểm mới so với đồng thuận thông thường;
- điều gì được chứng minh;
- điều gì được kiểm tra bằng mô phỏng;
- khả năng ứng dụng ở đâu.

### Bước 8 — Viết hạn chế

Kiểm tra:

- có phương pháp đối chứng không;
- có chỉ số định lượng không;
- có thử nghiệm phần cứng không;
- giả thiết đồ thị có mạnh không;
- có mô phỏng lỗi và mất gói tin không;
- kết luận có vượt quá dữ liệu không.

---

## 11. Checklist tự đánh giá

### Về mô hình

- Tác tử bậc một và bậc hai khác nhau thế nào?
- Leader và follower có vai trò gì?
- Nhiễu d_i(t) biểu diễn điều gì?

### Về đồ thị

- A, D, L là gì?
- Khi nào hai tác tử có cạnh?
- Vì sao trị riêng 0 quan trọng?
- Vì sao mạng ban đầu cần liên thông?

### Về đồng thuận

- Luật đồng thuận dùng thông tin gì?
- Consensus khác formation control thế nào?
- omega_i dùng để làm gì?
- Vì sao có thể xuất hiện nhiều nhóm?

### Về tránh vật cản

- Gradient hàm thế hướng tác tử như thế nào?
- Tránh vật cản làm đội hình biến dạng ra sao?
- Vì sao cần giới hạn vận tốc?

### Về ổn định

- Hàm Lyapunov đo điều gì?
- dot V nhỏ hơn hoặc bằng 0 cho biết gì?
- Vì sao cần LaSalle?
- Dwell time có ý nghĩa gì?

### Về mô phỏng

- Mỗi hình vẽ kiểm tra điều gì?
- Chỉ số nào cho biết đội hình tốt?
- Vì sao cần so sánh với phương pháp cơ sở?
- Mô phỏng hiện tại chưa kiểm tra loại lỗi nào?

---

## 12. Lộ trình học trong bốn tuần

### Tuần 1 — Nền tảng

- ngày 1–2: vector, ma trận, trị riêng;
- ngày 3–4: phương trình trạng thái và Euler;
- ngày 5: MATLAB/Octave;
- ngày 6–7: đồ thị, A, D, L.

Sản phẩm: mô phỏng 5 tác tử đồng thuận.

### Tuần 2 — Đồng thuận và hình thái

- ngày 8–9: consensus cơ bản;
- ngày 10: leader–follower;
- ngày 11–12: formation control;
- ngày 13: hình tròn, đường thẳng, tam giác;
- ngày 14: đọc phần Methods.

Sản phẩm: đội hình bám một leader chuyển động.

### Tuần 3 — An toàn và ổn định

- ngày 15–16: hàm thế;
- ngày 17: tránh tác tử;
- ngày 18: tránh vật cản;
- ngày 19–20: Lyapunov và LaSalle;
- ngày 21: topology thay đổi và dwell time.

Sản phẩm: đội hình tránh vật cản.

### Tuần 4 — Tái hiện và viết

- ngày 22–23: chạy 10 tác tử;
- ngày 24: chạy 25 tác tử;
- ngày 25: đo sai số và tỷ lệ kết nối;
- ngày 26: so sánh với bài báo;
- ngày 27: viết đóng góp và hạn chế;
- ngày 28: hoàn thiện và luyện trình bày.

Sản phẩm: hiểu mã, giải thích hình vẽ và bảo vệ báo cáo.

---

## 13. Những lỗi học tập thường gặp

### Chỉ đọc phần kết luận

Cách đọc tốt hơn:

```
Abstract
→ Introduction
→ Problem/model
→ Controller
→ Stability
→ Simulation
→ Conclusion
```

### Học thuộc phương trình nhưng không hiểu biến

Lập bảng:

| Ký hiệu | Ý nghĩa |
|---|---|
| x_i | vị trí tác tử |
| v_i | vận tốc |
| u_i | đầu vào điều khiển |
| a_ij | trọng số liên kết |
| L | ma trận Laplace |
| rho_i | offset hình thái |
| omega_i | hệ số nhóm |

### Nhầm mô phỏng với chứng minh

Mô phỏng chỉ kiểm tra một tập điều kiện. Nó không tự động chứng minh ổn định cho mọi điều kiện.

### Nhầm không đồng nhất với nhiều loại UAV

Không đồng nhất còn có thể đến từ mô hình, tham số, giới hạn vận tốc, cảm biến, năng lực tính toán hoặc vai trò khác nhau.

### Thay đổi quá nhiều tham số

Mỗi lần chỉ nên thay một tham số để biết nguyên nhân kết quả thay đổi.

---

## 14. Kết quả học tập cần đạt

Sau lộ trình, người học có thể:

1. trình bày bài toán bằng ngôn ngữ kỹ thuật;
2. giải thích mô hình bậc một/bậc hai;
3. xây dựng ma trận đồ thị;
4. mô tả luật đồng thuận;
5. giải thích hình thái bằng offset tương đối;
6. mô tả vai trò của hàm thế;
7. giải thích Lyapunov, LaSalle và dwell time;
8. chạy và chỉnh sửa mã;
9. đọc biểu đồ kết quả;
10. viết nhận xét có căn cứ;
11. phân biệt đóng góp thật với tuyên bố chưa kiểm chứng;
12. đề xuất thí nghiệm tiếp theo.

Quá trình tổng thể là:

```
kiến thức nền
→ mô hình toán
→ luật điều khiển
→ chứng minh ổn định
→ mô phỏng
→ phân tích kết quả
→ báo cáo khoa học
```

