window.HELP_IMPROVE_VIDEOJS = false;

// More Works Dropdown Functionality
function toggleMoreWorks() {
    const dropdown = document.getElementById('moreWorksDropdown');
    const button = document.querySelector('.more-works-btn');
    
    if (dropdown.classList.contains('show')) {
        dropdown.classList.remove('show');
        button.classList.remove('active');
    } else {
        dropdown.classList.add('show');
        button.classList.add('active');
    }
}

// Close dropdown when clicking outside
document.addEventListener('click', function(event) {
    const container = document.querySelector('.more-works-container');
    const dropdown = document.getElementById('moreWorksDropdown');
    const button = document.querySelector('.more-works-btn');
    
    if (container && !container.contains(event.target)) {
        dropdown.classList.remove('show');
        button.classList.remove('active');
    }
});

// Close dropdown on escape key
document.addEventListener('keydown', function(event) {
    if (event.key === 'Escape') {
        const dropdown = document.getElementById('moreWorksDropdown');
        const button = document.querySelector('.more-works-btn');
        dropdown.classList.remove('show');
        button.classList.remove('active');
    }
});

// Copy BibTeX to clipboard
function copyBibTeX() {
    const bibtexElement = document.getElementById('bibtex-code');
    const button = document.querySelector('.copy-bibtex-btn');
    const copyText = button.querySelector('.copy-text');
    
    if (bibtexElement) {
        navigator.clipboard.writeText(bibtexElement.textContent).then(function() {
            // Success feedback
            button.classList.add('copied');
            copyText.textContent = 'Cop';
            
            setTimeout(function() {
                button.classList.remove('copied');
                copyText.textContent = 'Copy';
            }, 2000);
        }).catch(function(err) {
            console.error('Failed to copy: ', err);
            // Fallback for older browsers
            const textArea = document.createElement('textarea');
            textArea.value = bibtexElement.textContent;
            document.body.appendChild(textArea);
            textArea.select();
            document.execCommand('copy');
            document.body.removeChild(textArea);
            
            button.classList.add('copied');
            copyText.textContent = 'Cop';
            setTimeout(function() {
                button.classList.remove('copied');
                copyText.textContent = 'Copy';
            }, 2000);
        });
    }
}

// Scroll to top functionality
function scrollToTop() {
    window.scrollTo({
        top: 0,
        behavior: 'smooth'
    });
}

// Show/hide scroll to top button
window.addEventListener('scroll', function() {
    const scrollButton = document.querySelector('.scroll-to-top');
    if (window.pageYOffset > 300) {
        scrollButton.classList.add('visible');
    } else {
        scrollButton.classList.remove('visible');
    }
});

// Video carousel autoplay when in view
function setupVideoCarouselAutoplay() {
    const carouselVideos = document.querySelectorAll('.results-carousel video');
    
    if (carouselVideos.length === 0) return;
    
    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            const video = entry.target;
            if (entry.isIntersecting) {
                // Video is in view, play it
                video.play().catch(e => {
                    // Autoplay failed, probably due to browser policy
                    console.log('Autoplay prevented:', e);
                });
            } else {
                // Video is out of view, pause it
                video.pause();
            }
        });
    }, {
        threshold: 0.5 // Trigger when 50% of the video is visible
    });
    
    carouselVideos.forEach(video => {
        observer.observe(video);
    });
}

$(document).ready(function() {
    // Check for click events on the navbar burger icon

    var options = {
		slidesToScroll: 1,
		slidesToShow: 1,
		loop: true,
		infinite: true,
		autoplay: true,
		autoplaySpeed: 5000,
    }

	// Initialize all div with carousel class
    var carousels = bulmaCarousel.attach('.carousel', options);
	
    bulmaSlider.attach();
    
    // Setup video autoplay for carousel
    setupVideoCarouselAutoplay();

    // document.addEventListener('DOMContentLoaded', () => {
    
    //     // 初始化轮播图 (你原本的代码)
    //     var options = { slidesToScroll: 1, slidesToShow: 1, loop: true, infinite: true, autoplay: true, autoplaySpeed: 5000 };
    //     if (typeof bulmaCarousel !== 'undefined') {
    //         var carousels = bulmaCarousel.attach('.carousel', options);
    //     }
    //     if (typeof bulmaSlider !== 'undefined') {
    //         bulmaSlider.attach();
    //     }
    
    //     /* ==============================================================
    //        功能 1: 表格切换逻辑 (Scope: #table-tabs-system)
    //        ============================================================== */
    //     function initTableTabs() {
    //         const container = document.getElementById('table-tabs-system');
    //         if (!container) return; // 如果找不到容器就停止，防止报错
    
    //         // 只查找此容器内的元素
    //         const tabs = container.querySelectorAll('.tabs li');
    //         const contents = container.querySelectorAll('.my-tab-content');
    
    //         tabs.forEach(tab => {
    //             tab.addEventListener('click', (e) => {
    //                 e.preventDefault(); // 阻止链接跳转
                    
    //                 // 1. 移除此容器内所有的激活状态
    //                 tabs.forEach(t => t.classList.remove('is-active'));
    //                 contents.forEach(c => c.style.display = 'none');
    
    //                 // 2. 激活当前点击的
    //                 tab.classList.add('is-active');
                    
    //                 // 3. 显示对应内容
    //                 const targetId = tab.dataset.target;
    //                 const targetContent = container.querySelector(`#${targetId}`);
    //                 if (targetContent) {
    //                     targetContent.style.display = 'block';
    //                 }
    //             });
    //         });
    //     }
    
    //     /* ==============================================================
    //        功能 2: 图片轮播切换逻辑 (Scope: #wheel-tab-system)
    //        ============================================================== */
    //     function initWheelTabs() {
    //         const container = document.getElementById('wheel-tab-system');
    //         if (!container) return;
    
    //         // 只查找此容器内的元素
    //         const dots = container.querySelectorAll('.indicator-dot');
    //         const wrapper = container.querySelector('.wheel-tabs-wrapper');
    //         const tabs = container.querySelectorAll('.wheel-tab');
    
    //         dots.forEach((dot, index) => {
    //             dot.addEventListener('click', (e) => {
    //                 e.preventDefault();
    //                 e.stopPropagation(); // 关键：防止事件冒泡影响其他组件
    
    //                 // 1. 更新圆点状态
    //                 dots.forEach(d => d.classList.remove('active'));
    //                 dot.classList.add('active');
    
    //                 // 2. 移动滑动容器 (transform)
    //                 if (wrapper) {
    //                     wrapper.style.transform = `translateX(-${index * 100}%)`;
    //                 }
    
    //                 // 3. 更新 Tab 激活状态 (用于处理淡入淡出动画)
    //                 tabs.forEach((tab, tabIndex) => {
    //                     if (tabIndex === index) {
    //                         tab.classList.add('active');
    //                         tab.style.opacity = '1';
    //                     } else {
    //                         tab.classList.remove('active');
    //                         tab.style.opacity = '0';
    //                     }
    //                 });
    //             });
    //         });
    //     }
    
    //     // 执行两个初始化函数
    //     initTableTabs();
    //     initWheelTabs();
    // });

    // 设置表格标签页切换功能
function setupTableTabs() {
    const tabs = document.querySelectorAll('.tabs li');
    const tabContent = document.querySelectorAll('.tab-content');

    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            // 1. 移除所有标签页和内容的激活状态
            tabs.forEach(item => item.classList.remove('is-active'));
            tabContent.forEach(item => item.style.display = 'none');

            // 2. 激活被点击的标签页
            tab.classList.add('is-active');

            // 3. 获取目标内容的ID并显示它
            const targetId = tab.dataset.target; // 获取 data-target="table-x-content"
            const targetContent = document.getElementById(targetId);
            if (targetContent) {
                targetContent.style.display = 'block';
            }
        });
    });
}

// 确保在页面加载完成后执行 setupTableTabs
$(document).ready(function() {
    // 检查是否有 tabs 结构存在，然后再设置监听器
    if (document.querySelector('.tabs li')) {
        setupTableTabs();
    }
    
    // 如果之前有 load('static/table/tuihua.html', ...) 的代码，请在这里删除或注释掉。
});

  
})
