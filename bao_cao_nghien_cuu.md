# BÁO CÁO NGHIÊN CỨU CÔNG TRÌNH KHOA HỌC

## Điều khiển hình thái dựa trên sự đồng thuận cho các hệ đa tác tử không đồng nhất trong môi trường phức tạp

### 1. Thông tin chung về công trình

- **Tên bài báo:** *Consensus-Based Formation Control for Heterogeneous Multi-Agent Systems in Complex Environments*
- **Tác giả:** Xiaofei Chang, Yiming Yang, Zhuo Zhang, Jiayue Jiao, Haoyu Cheng và Wenxing Fu.
- **Nguồn:** *Drones*, 2025, 9(3), 175.
- **Đối tượng nghiên cứu:** hệ đa tác tử, tập trung vào các phương tiện bay không người lái (UAV) có động học khác nhau.
- **Phương pháp trung tâm:** điều khiển hình thái phân tán dựa trên lý thuyết đồng thuận, lý thuyết đồ thị, đồng thuận đa nhóm và phân tích ổn định Lyapunov.

## 2. Bài toán và vấn đề được xử lý

### 2.1. Vấn đề chính

Bài báo giải quyết bài toán điều khiển đội hình của một hệ đa tác tử **không đồng nhất** trong môi trường phức tạp và biến đổi. Các tác tử không nhất thiết có cùng mô hình động lực học: một số tác tử được mô tả bằng mô hình bậc một, trong khi một số khác có mô hình bậc hai, có thêm trạng thái vận tốc và gia tốc.

Mục tiêu của hệ là:

1. duy trì khoảng cách và vị trí tương đối mong muốn giữa các tác tử;
2. cho phép các tác tử đạt đồng thuận trong từng nhóm;
3. tránh va chạm giữa các tác tử và tránh chướng ngại vật;
4. duy trì kết nối truyền thông khi có thể;
5. thay đổi hình dạng đội hình và cấu trúc nhóm khi môi trường hoặc nhiệm vụ thay đổi;
6. bảo đảm tính ổn định khi đồ thị truyền thông thay đổi theo thời gian.

Nói ngắn gọn, bài toán là thiết kế một luật điều khiển phân tán để các tác tử chỉ sử dụng thông tin cục bộ nhưng vẫn đạt được hành vi đội hình có tổ chức trong môi trường phức tạp.

### 2.2. Bối cảnh

Bối cảnh được tác giả nhấn mạnh là các nhiệm vụ UAV trong khu vực đô thị có nhiều nhà cao tầng, khe hẹp, ngõ hẹp, vật cản tĩnh hoặc chuyển động; ngoài ra còn có thể mở rộng sang tìm kiếm-cứu nạn ở địa hình núi. Trong những môi trường này, đội hình phải liên tục điều chỉnh để đi qua không gian hẹp nhưng không được mất cấu trúc phối hợp.

Hệ đồng nhất có thể áp dụng một luật điều khiển giống nhau cho mọi tác tử, nhưng hệ không đồng nhất khó hơn vì các tác tử có khả năng cảm nhận, tốc độ, mô hình động lực học và vai trò khác nhau. Một tác tử bậc một có thể được xem là có khả năng điều chỉnh trực tiếp vị trí, còn tác tử bậc hai phải điều khiển thông qua vận tốc và gia tốc.

### 2.3. Vì sao bài toán quan trọng?

Điều khiển tập trung có thể tạo ra phối hợp chính xác nhưng có nhược điểm là khó mở rộng, phụ thuộc vào một nút trung tâm và dễ bị ảnh hưởng bởi điểm lỗi đơn. Điều khiển phân tán có khả năng mở rộng và chịu lỗi tốt hơn, nhưng khó bảo đảm rằng các quyết định cục bộ vẫn tạo ra đội hình ổn định ở cấp toàn hệ thống.

Một phương pháp phù hợp có ý nghĩa thực tế đối với:

- tìm kiếm và cứu hộ;
- giám sát, trinh sát và kiểm tra công trình;
- vận chuyển hoặc logistics bằng UAV;
- bay qua khu vực đô thị có mật độ vật cản cao.

Bài báo đặt mục tiêu đồng thời cải thiện tính tự chủ, khả năng phối hợp, độ tin cậy và khả năng thích nghi của hệ đa tác tử.

## 3. Phương pháp tác giả đề xuất

### 3.1. Mô hình mạng bằng lý thuyết đồ thị

Mỗi tác tử được biểu diễn bởi một nút của đồ thị truyền thông, còn liên kết giữa hai tác tử là một cạnh. Tác giả sử dụng đồ thị vô hướng có trọng số

\[
G=(V,E,A), \qquad L=D-A,
\]

trong đó `A` là ma trận kề, `D` là ma trận bậc và `L` là ma trận Laplace. Khi tác tử ở xa nhau, trọng số liên kết giảm dần theo khoảng cách; khi vượt quá bán kính truyền thông `R`, liên kết được xem là mất. Hàm giảm trơn dạng cosin được sử dụng để tránh sự thay đổi đột ngột của trọng số.

Trong mô hình có lãnh đạo, ma trận kết nối với leader được thêm vào Laplace để tạo ma trận `H=L+B`. Leader lập quỹ đạo và cập nhật đường đi; followers sử dụng thông tin từ leader và các tác tử lân cận để bám đội hình.

### 3.2. Mô hình động lực học không đồng nhất

Hệ gồm `N` followers và `M` leaders. Leader được mô tả bởi mô hình bậc hai:

\[
\dot{x}_i=v_i, \qquad \dot{v}_i=u_{i0}.
\]

Với followers, tác giả xét hai loại:

- **Tác tử bậc một:**

  \[
  \dot{x}_i=u_i+f_i(x_i)+d_i(t),
  \]

  trong đó `f_i` biểu diễn thành phần động lực học và `d_i` là nhiễu/tác động bên ngoài.

- **Tác tử bậc hai:**

  \[
  \dot{x}_i=v_i, \qquad \dot{v}_i=u_i+d_i(t).
  \]

Như vậy, bộ điều khiển phải làm việc được với các tác tử có mức độ tự chủ và động lực học khác nhau.

### 3.3. Đồng thuận đa nhóm và hệ số mức độ thông minh

Khác với đồng thuận thông thường, trong đó tất cả tác tử hội tụ về một trạng thái chung, bài báo sử dụng **đồng thuận đa nhóm**. Các tác tử được chia thành các nhóm; tác tử trong cùng nhóm hội tụ về cùng một quan hệ hình thái, còn các nhóm khác nhau có thể có mục tiêu hoặc vị trí tương đối khác nhau.

Tác giả đưa vào hệ số mức độ thông minh `ω_i`. Các tác tử có cùng giá trị `ω_i` được khuyến khích đạt cùng một trạng thái đồng thuận; các giá trị khác nhau tạo ra các nhóm với mục tiêu hội tụ khác nhau. Ma trận Laplace được hiệu chỉnh theo các hệ số này, tạo ra ma trận Laplace cho đồng thuận đa nhóm.

Cách làm này cho phép hệ:

- tự tổ chức thành các nhóm phục vụ các mục tiêu khác nhau;
- thay đổi số lượng và cấu trúc nhóm;
- duy trì phối hợp nội nhóm ngay cả khi không phải mọi tác tử đều liên lạc trực tiếp với nhau;
- thay đổi hình thái theo yêu cầu nhiệm vụ.

### 3.4. Luật điều khiển hình thái

Luật điều khiển cốt lõi trong bài báo kết hợp bốn thành phần:

1. **Thành phần tránh va chạm:** dùng hàm thế năng `G_1`. Khi khoảng cách giữa hai tác tử nhỏ hơn ngưỡng an toàn, lực đẩy tăng lên; tác tử càng gần nhau thì tác động tránh va chạm càng lớn.

2. **Thành phần duy trì kết nối và khoảng cách hình thái:** dùng hàm thế năng `G_2`. Thành phần này giữ khoảng cách giữa các tác tử trong vùng cho phép, đồng thời khuyến khích chúng tiến tới khoảng cách tương đối mong muốn `ρ_ij=ρ_i-ρ_j`. Khi khoảng cách vượt quá bán kính truyền thông, liên kết có thể bị ngắt để hệ tái cấu hình linh hoạt.

3. **Thành phần đồng thuận:** sử dụng sai số vị trí chuẩn hóa theo `ω_i` và hàm `S(·)`. Thành phần này làm giảm sai số giữa các tác tử cùng nhóm và đưa chúng về mục tiêu hình thái.

4. **Thành phần hãm vận tốc:** có dạng `-βv_i`, giúp giảm năng lượng động học và tránh dao động quá mức khi tác tử tiếp cận vị trí đích.

Ở dạng khái quát, luật điều khiển trong công thức (13) của bài báo có thể viết:

\[
u_i=-\omega_i^2\sum_{j\in N_i^q}\nabla_{x_i}G_1
-\omega_i^2\sum_{j\in N_i^l}\nabla_{x_i}G_2
-\alpha\sum_{j\in N_i}S(\varepsilon_i^\rho,\varepsilon_j^\rho,v_m)
-\beta v_i.
\]

Trong đó `α, β>0`, `v_m` là vận tốc an toàn cực đại, còn `ε_i^ρ=x_i-ρ_i` là sai số vị trí tương ứng với hình thái đích.

### 3.5. Bộ điều khiển tránh chướng ngại vật và thay đổi hình thái

Trong phần thiết kế điều khiển phối hợp, tác giả tách đầu vào thành:

\[
u_i=-u_{ij}-u_{iv}-\beta V_C(:,i),
\]

với:

- `u_ij`: điều khiển cơ sở đưa tác tử về vị trí đích hoặc vị trí tương đối đích;
- `u_iv`: thành phần tránh va chạm/chướng ngại vật dựa trên khoảng cách `d_ij`;
- `V_C=-K_vv_i`: thành phần hạn chế vận tốc.

Các ngưỡng khoảng cách được nối bằng hàm cosin trơn. Điều này làm cho lực tránh vật cản tăng dần khi tác tử tiến vào vùng nguy hiểm, thay vì tạo ra một lệnh điều khiển nhảy đột ngột.

### 3.6. Phân tích ổn định

Tác giả xây dựng hàm Lyapunov gồm năng lượng của các hàm thế tránh va chạm, duy trì kết nối, sai số đồng thuận và thành phần vận tốc. Sau khi thay luật điều khiển vào đạo hàm của hàm Lyapunov, bài báo thu được kết quả chính:

\[
\dot{V}=-\beta\hat v^{\mathsf T}\hat v\leq 0.
\]

Do đó, hàm Lyapunov không tăng và hệ không rời khỏi miền giới hạn ban đầu nếu các điều kiện về khoảng cách được thỏa mãn. Tác giả tiếp tục dùng nguyên lý bất biến LaSalle để suy ra vận tốc hội tụ về 0 và các tác tử đạt trạng thái đồng thuận ổn định.

Đối với đồ thị thay đổi theo thời gian, tác giả đưa vào **thời gian lưu tối thiểu** (minimum dwell time). Nếu hai lần chuyển mạng xảy ra tại `t_k` và `t_{k+1}`, thì:

\[
\tau(t_k)=t_{k+1}-t_k\geq \tau_d>0.
\]

Giả thiết này bảo đảm mỗi cấu hình mạng tồn tại đủ lâu để hệ ổn định trước khi mạng chuyển sang cấu hình kế tiếp. Bài báo cũng yêu cầu mạng ban đầu liên thông đầy đủ và chọn các ngưỡng thế năng thích hợp để tránh mất kết nối ngoài ý muốn.

## 4. Kết quả chính đạt được

### 4.1. Kết quả lý thuyết

Kết quả lý thuyết được trình bày theo hai trường hợp:

- **Mạng tĩnh:** luật đồng thuận đa nhóm làm giảm hàm Lyapunov; hệ đạt ổn định tiệm cận trong điều kiện các hệ số thông minh khác 0, năng lượng Lyapunov ban đầu hữu hạn và khoảng cách hình thái nằm trong vùng cho phép.
- **Mạng động:** với giả thiết thời gian lưu tối thiểu đủ lớn, các lần chuyển đổi topology vẫn giữ được tính ổn định của hệ.

Kết quả này cho phép hình thái được thay đổi theo từng đoạn liên tục thay vì cố định trong toàn bộ nhiệm vụ. Các hàm thế cũng cung cấp cơ chế giữ khoảng cách an toàn và hỗ trợ duy trì liên lạc.

### 4.2. Kiểm chứng đồng thuận của hệ 10 tác tử

Thí nghiệm mô phỏng đầu tiên gồm 10 tác tử:

- tác tử 1–6 là hệ bậc một;
- tác tử 7–10 là hệ bậc hai;
- leader cung cấp thông tin vị trí cho followers;
- mô phỏng được thực hiện bằng MATLAB 2022b.

Các hình 1–5 thể hiện lần lượt quỹ đạo trạng thái, vận tốc của tác tử bậc một, vị trí, vận tốc và gia tốc của tác tử bậc hai. Theo bài báo, hệ đạt trạng thái ổn định tại thời điểm định trước khoảng `t=5 s`, các followers bám thông tin trạng thái của leader và sai lệch giữa các tác tử giảm về trạng thái đồng thuận.

### 4.3. Kiểm chứng hình thái và tránh vật cản

Trong thí nghiệm tiếp theo, hệ gồm 10 tác tử, trong đó có 6 tác tử bậc một và 4 tác tử bậc hai. Các tác tử phải duy trì đội hình khi đi qua môi trường có vật cản và thay đổi hình dạng theo yêu cầu vận hành.

Các tham số chính trong Bảng 2 của bài báo:

| Tham số | Giá trị |
|---|---:|
| Bán kính truyền thông `R` | 50 m |
| Biên giới hình thái `R'` | 40 m |
| Biên ngoài vùng loại trừ `r_out` | 5 m |
| Biên trong vùng loại trừ `r_in` | 0,5 m |
| Hệ số hàm thế `(k_1,k_2)` | (8, 2) |
| Tham số điều khiển `(α,β)` | (1,5; 1,5) |
| Mức độ thông minh `ω` | (1; 1,1; 1,2) |
| Tốc độ suy giảm truyền thông `τ` | 0,6 |
| Cận dưới kết nối `ε` | 0,4 |

Kết quả ở Hình 7 cho thấy đội hình lần lượt chuyển từ dạng tròn sang dạng đường thẳng và dạng tam giác. Sự thay đổi này được giải thích bởi yêu cầu đi qua khe hẹp và tránh chướng ngại vật. Tác tử bậc một dùng mô hình đơn giản hơn để duy trì đội hình và đi dọc theo biên vật cản; tác tử bậc hai có động lực học chi tiết hơn nên phù hợp với việc điều khiển chuyển động chính xác qua các khoảng hẹp.

Hình 8 thể hiện biến thiên vị trí `x` và `y` của ba nhóm tác tử bậc một/bậc hai. Các đường cong cho thấy hệ có thể thay đổi đồng thuận theo thời gian và tái cấu hình hình thái trong môi trường động.

### 4.4. Mô phỏng mở rộng với 25 tác tử

Để kiểm tra khả năng mở rộng, bài báo mô phỏng 25 tác tử chia thành bốn nhóm. Môi trường có các vật cản chuyển động dạng tam giác, hình chữ nhật và dạng kéo dài. Kết quả Hình 9 cho thấy:

- các nhóm hình thành đội hình ban đầu;
- các nhóm vẫn giữ được tính nhất quán nội nhóm;
- hệ tránh được các vật cản chuyển động;
- hình thái được điều chỉnh qua nhiều giai đoạn;
- các tác tử bậc một và bậc hai thể hiện ưu thế khác nhau trong các nhiệm vụ điều khiển.

Tuy nhiên, bài báo chủ yếu trình bày kết quả dưới dạng quỹ đạo và nhận xét trực quan. Ngoài mốc khoảng 5 giây trong kiểm chứng đồng thuận, bài báo chưa cung cấp đầy đủ sai số RMS, khoảng cách tránh nhỏ nhất, năng lượng điều khiển, tỷ lệ thành công hoặc so sánh số liệu với một phương pháp chuẩn.

## 5. Đóng góp chính của công trình

### 5.1. Đóng góp về phương pháp

Đóng góp quan trọng nhất là kết hợp các thành phần thường được nghiên cứu riêng rẽ thành một chiến lược thống nhất:

- điều khiển phân tán dựa trên đồng thuận;
- lý thuyết đồ thị cho mạng truyền thông;
- đồng thuận đa nhóm thông qua hệ số `ω_i`;
- hàm thế tránh va chạm và duy trì kết nối;
- điều khiển hình thái thay đổi theo thời gian;
- chứng minh ổn định với Lyapunov, LaSalle và thời gian lưu tối thiểu.

### 5.2. Điểm mới so với cách tiếp cận thông thường

So với điều khiển đội hình đồng nhất và cố định, phương pháp đề xuất có ba điểm khác biệt đáng chú ý:

1. **Xử lý trực tiếp tính không đồng nhất:** hệ có thể gồm tác tử bậc một và bậc hai thay vì giả sử mọi tác tử có cùng động lực học.
2. **Cho phép nhiều nhóm và nhiều mục tiêu:** các giá trị mức độ thông minh khác nhau tạo ra các nhóm đồng thuận khác nhau.
3. **Xét topology thay đổi:** cấu trúc liên kết có thể thay đổi khi khoảng cách và hình thái thay đổi, với điều kiện chuyển mạng được kiểm soát bởi thời gian lưu tối thiểu.

### 5.3. Tiềm năng ứng dụng

Phương pháp phù hợp với đội UAV phân tán trong tìm kiếm-cứu nạn, giám sát đô thị, kiểm tra cầu/đường dây, lập bản đồ và logistics. Trong các nhiệm vụ này, nhiều UAV có thể đảm nhiệm vai trò khác nhau: một số có cảm biến và năng lực tính toán tốt hơn làm leader, số còn lại đảm nhiệm quan sát, dẫn đường hoặc bao phủ khu vực.

## 6. Nhận xét và đánh giá

### 6.1. Ưu điểm

1. **Bài toán có ý nghĩa thực tế cao.** Tác giả lựa chọn một tình huống khó và gần với nhu cầu UAV thực tế: duy trì đội hình trong môi trường nhiều vật cản và có khe hẹp.
2. **Kiến trúc điều khiển có tính phân tán.** Mỗi tác tử chủ yếu dùng thông tin cục bộ, nhờ đó phù hợp hơn với hệ lớn và giảm phụ thuộc vào trung tâm.
3. **Kết hợp được nhiều mục tiêu điều khiển.** Phương pháp không chỉ làm giảm sai số hình thái mà còn xét tránh va chạm, duy trì kết nối và giới hạn vận tốc.
4. **Có phân tích ổn định.** Việc dùng hàm Lyapunov, nguyên lý LaSalle và thời gian lưu tối thiểu tạo cơ sở toán học cho trường hợp mạng tĩnh và mạng chuyển mạch.
5. **Có kiểm thử nhiều quy mô.** Tác giả mô phỏng hệ 10 tác tử không đồng nhất và mở rộng lên 25 tác tử chia thành bốn nhóm.

### 6.2. Hạn chế và điểm cần thận trọng

1. **Thiếu đối chứng định lượng.** Bài báo không so sánh trực tiếp với điều khiển leader–follower, đồng thuận chuẩn, MPC hoặc một phương pháp phân tán hiện có. Vì vậy, kết luận rằng phương pháp “cải thiện đáng kể” chưa được củng cố bằng số liệu đối chứng.
2. **Chưa đo lường đầy đủ hiệu năng.** Cần có sai số hình thái theo thời gian, thời gian hội tụ, khoảng cách vật cản nhỏ nhất, số lần mất kết nối, năng lượng điều khiển và tỷ lệ hoàn thành nhiệm vụ.
3. **Tuyên bố về chịu lỗi và chống gây nhiễu chưa được kiểm tra đầy đủ.** Phần giới thiệu nêu khả năng chịu lỗi/chống gây nhiễu, nhưng các mô phỏng được mô tả chưa cho thấy thí nghiệm cụ thể về hỏng actuator, mất gói tin, trễ truyền thông hoặc jamming.
4. **Giả thiết mạng ban đầu khá mạnh.** Mạng ban đầu được yêu cầu liên thông đầy đủ và đồ thị là vô hướng. Trong UAV thực tế, liên kết thường có hướng, không đối xứng và có thể mất ngẫu nhiên.
5. **Giả thiết môi trường còn lý tưởng.** Quỹ đạo tham chiếu được giả sử trơn, nhiễu bị chặn và nhiễu ban đầu nhỏ. Chưa có đánh giá hệ thống với cảm biến nhiễu mạnh, vật cản có tốc độ cao hoặc thay đổi bất ngờ.
6. **Mối liên hệ giữa mô hình lý thuyết và vật cản môi trường cần làm rõ hơn.** Hàm thế được trình bày rõ cho quan hệ giữa các tác tử, nhưng phần mô hình hóa vật cản bên ngoài và động lực của vật cản chuyển động chưa được trình bày đầy đủ.
7. **Chưa có thử nghiệm phần cứng hoặc bay thực.** Mô phỏng MATLAB là bước kiểm tra cần thiết, nhưng chưa phản ánh đầy đủ giới hạn tính toán, sai số định vị, độ trễ cảm biến và các ràng buộc của UAV thật.
8. **Một số khái niệm chưa hoàn toàn thống nhất.** Bài báo có đưa ra định nghĩa về điều khiển thời gian đặt trước, nhưng phần chứng minh chính chủ yếu dẫn tới hội tụ tiệm cận. Nếu muốn khẳng định hội tụ tại thời gian đặt trước, cần có chặn thời gian hội tụ và chứng minh tương ứng.
9. **Một số ký hiệu và phương trình khó theo dõi.** Kích thước vector, chỉ số leader/follower và cách đưa thành phần động lực học `f_i(x_i), d_i(t)` vào chứng minh cần được chuẩn hóa để người đọc dễ tái lập kết quả.

### 6.3. Đề xuất cải thiện

Để nâng cao giá trị khoa học và khả năng ứng dụng, nghiên cứu có thể được mở rộng theo các hướng sau:

- thực hiện so sánh với ít nhất ba đường cơ sở: leader–follower, đồng thuận không phân nhóm và MPC/điều khiển tránh vật cản;
- báo cáo các chỉ số: sai số hình thái RMS, sai số lớn nhất, thời gian hội tụ, khoảng cách an toàn nhỏ nhất, năng lượng điều khiển, tỷ lệ duy trì kết nối và tỷ lệ hoàn thành nhiệm vụ;
- thực hiện thí nghiệm loại bỏ từng thành phần để đánh giá riêng tác dụng của đồng thuận đa nhóm, hàm thế, hệ số `ω_i` và cơ chế chuyển topology;
- mô phỏng lỗi actuator, mất gói tin, trễ truyền thông, nhiễu cảm biến, jamming và tác tử rời mạng;
- mở rộng từ đồ thị vô hướng sang đồ thị có hướng và mạng thay đổi ngẫu nhiên;
- mô hình hóa rõ vật cản chuyển động, vận tốc tương đối và giới hạn gia tốc của từng UAV;
- dùng điều khiển dựa trên hàm rào an toàn hoặc kiểm soát dự đoán để bảo đảm an toàn trong tình huống vật cản xuất hiện đột ngột;
- kiểm chứng bằng mô phỏng phần cứng trong vòng lặp và bay thực với PX4/ROS hoặc nền tảng tương đương;
- công bố mã nguồn, điều kiện ban đầu, quỹ đạo vật cản và dữ liệu mô phỏng để tăng khả năng tái lập.

## 7. Kết luận chung

Bài báo đề xuất một khung điều khiển hình thái phân tán có khả năng xử lý hệ đa tác tử không đồng nhất, nhiều nhóm và topology truyền thông thay đổi. Ý tưởng chính là dùng hệ số mức độ thông minh để tạo đồng thuận đa nhóm, kết hợp hàm thế cho tránh va chạm và duy trì kết nối, sau đó chứng minh ổn định bằng Lyapunov, LaSalle và điều kiện thời gian lưu tối thiểu.

Các mô phỏng 10 và 25 tác tử cho thấy phương pháp có thể duy trì đội hình, thay đổi giữa dạng tròn, đường thẳng và tam giác, đồng thời tránh các vật cản tĩnh/chuyển động trong kịch bản mô phỏng. Đây là một nền tảng tốt cho điều khiển đội UAV trong môi trường phức tạp.

Tuy nhiên, bằng chứng hiện tại chủ yếu là lý thuyết và mô phỏng định tính. Để khẳng định mạnh hơn về tính ưu việt, độ bền trước lỗi và khả năng triển khai thực tế, cần bổ sung chỉ số định lượng, phương pháp đối chứng, các kịch bản mất liên lạc/hỏng hóc và thí nghiệm trên phần cứng.

### Tài liệu tham khảo chính

Chang, X.; Yang, Y.; Zhang, Z.; Jiao, J.; Cheng, H.; Fu, W. *Consensus-Based Formation Control for Heterogeneous Multi-Agent Systems in Complex Environments*. **Drones** 2025, 9, 175. DOI: 10.3390/drones9030175.
